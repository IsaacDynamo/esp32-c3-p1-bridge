use embedded_svc::wifi::*;
use esp_idf_svc::eventloop::*;
use esp_idf_svc::netif::*;
use esp_idf_svc::wifi::*;

use anyhow::{bail, Result};
use log::*;

use std::{net::Ipv4Addr, sync::atomic::Ordering, thread::sleep, time::Duration};

const SSID: &str = include_str!("../ssid.txt");
const PASSWORD: &str = include_str!("../password.txt");

pub fn wifi_connection(mut wifi: Box<EspWifi<'static>>, sysloop: EspSystemEventLoop) -> Result<()> {
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    }))?;

    wifi.start()?;

    info!("Starting wifi...");

    if !WifiWait::new(&sysloop)?
        .wait_with_timeout(Duration::from_secs(20), || wifi.is_started().unwrap())
    {
        bail!("Wifi did not start");
    }

    info!("Connecting wifi...");

    wifi.connect()?;

    if !EspNetifWait::new::<EspNetif>(wifi.sta_netif(), &sysloop)?
        .wait_with_timeout(Duration::from_secs(20), || wifi.is_connected().unwrap())
    {
        bail!("Wifi did not connect");
    }

    info!("Get DHCP lease...");

    if !EspNetifWait::new::<EspNetif>(wifi.sta_netif(), &sysloop)?
        .wait_with_timeout(Duration::from_secs(20), || {
            wifi.sta_netif().get_ip_info().unwrap().ip != Ipv4Addr::new(0, 0, 0, 0)
        })
    {
        bail!("Did not receive a DHCP lease");
    }

    let ip_info = wifi.sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {:?}", ip_info);

    crate::WIFI_STATUS.store(1, Ordering::Relaxed);

    loop {
        sleep(Duration::from_millis(1000));
    }
}
