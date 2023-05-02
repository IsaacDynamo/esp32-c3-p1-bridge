//use esp_idf_hal::peripheral;
use esp_idf_svc::eventloop::*;
use esp_idf_svc::netif::*;
//use esp_idf_svc::nvs::*;
//use esp_idf_svc::ping;
//use esp_idf_svc::systime::EspSystemTime;
use anyhow::{bail, Result};
//use embedded_svc::ipv4;
use embedded_svc::wifi::*;
use esp_idf_svc::wifi::*;
use log::*;
use std::net::Ipv4Addr;
use std::sync::atomic::Ordering;

use std::{thread::sleep, time::Duration};

const SSID: &str = include_str!("../ssid.txt");
const PASSWORD: &str = include_str!("../password.txt");

pub fn wifi_connection(mut wifi: Box<EspWifi<'static>>, sysloop: EspSystemEventLoop) -> Result<()> {
    //let sysloop = EspSystemEventLoop::take()?;
    //let mut wifi = Box::new(EspWifi::new(modem, sysloop.clone(), None)?);

    info!("Wifi created, about to scan");

    fn scan(wifi: &mut Box<EspWifi<'static>>) -> Result<u8> {
        loop {
            let ap_infos = wifi.scan()?;
            let ours = ap_infos.into_iter().find(|a| a.ssid == SSID);
            if let Some(ours) = ours {
                return Ok(ours.channel);
            } else {
                sleep(Duration::from_millis(1000));
            }
        }
    }

    let channel = scan(&mut wifi)?;
    info!(
        "Found configured access point {} on channel {}",
        SSID, channel
    );

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        channel: Some(channel),
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

    if !EspNetifWait::new::<EspNetif>(wifi.sta_netif(), &sysloop)?.wait_with_timeout(
        Duration::from_secs(20),
        || {
            wifi.is_connected().unwrap()
                && wifi.sta_netif().get_ip_info().unwrap().ip != Ipv4Addr::new(0, 0, 0, 0)
        },
    ) {
        bail!("Wifi did not connect or did not receive a DHCP lease");
    }

    let ip_info = wifi.sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {:?}", ip_info);

    //ping(ip_info.subnet.gateway)?;

    crate::WIFI_STATUS.store(1, Ordering::Relaxed);

    loop {
        sleep(Duration::from_millis(1000));
    }
}

// fn ping(ip: ipv4::Ipv4Addr) -> Result<()> {
//     info!("About to do some pings for {:?}", ip);

//     let ping_summary = ping::EspPing::default().ping(ip, &Default::default())?;
//     if ping_summary.transmitted != ping_summary.received {
//         bail!("Pinging IP {} resulted in timeouts", ip);
//     }

//     info!("Pinging done");

//     Ok(())
// }
