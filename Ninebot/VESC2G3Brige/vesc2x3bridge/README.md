# VESC2X3Bridge

This repository provides the PCB files for the VESC2X3Bridge board. It allows an almost plug-and-play installation, with minimal configuration required on both the VESC and Bridge sides.

Community Telegram channel [Ninebot Bridges](https://t.me/NinebotBridges)

[Demo Video with my Firmware](https://youtu.be/Sf85hNRvy-E)

Also check out this repository which is used to drive the lights on your X3 Series Scooter [LightBoard](https://gitea.overkill.cc/morss12/LightBoard.git)

# Features & Abilities

A simple overview of everything the VESC2G3Bridge can do.

## What Is It?

The VESC2X3Bridge is a project I started at the end of 2025. It aims to integrate the X3 Scooter Series from Ninebot into the world of VESC, an open-source project by Benjamin Vedder.
The VESC2G3Bridge is a small computer that sits between your scooter's motor controller (VESC) and its dashboard (G3). Without it, the dashboard and the motor controller don't understand each other — the bridge acts like a translator so they can work together.

## What It Does

### Shows Your Speed
The device reads how fast the motor is spinning and works out your actual riding speed. It then sends that number to the dashboard so you can see it while you ride.

### Shows Battery Level
It checks how much battery you have left and tells the dashboard, so you always know when to charge.

### Controls the Lights
- Turns on the **rear light** just like the stock X3 Series.
- Flashes the **left and right indicators** (turn signals) when you signal a turn.
- Supports **horn** activation from the dashboard button (GT3).

### Switches Ride Profiles
The scooter can have different power modes (WALK,ECO,NORMAL,SPORT). The bridge listens to the dashboard buttons and tells the VESC which mode to switch to. You can also set it to block the switching of modes when the battery is low, bypassing the dashboards auto apply of a slow profile.

### Controls the Throttle and Brake
By running a small script on the VESC, the VESC can directly read the inputs from the dashboard, giving the VESC full control over how you accelerate and slow down.

### Updates Itself Over Wi-Fi
The device creates its own Wi-Fi network and you can upload new firmware right from your phone or computer's web browser.

### Has a Built-In Settings Page
Connect to its Wi-Fi and open a web page to change settings. [Config page](https://192.168.5.1/)

### Works With Different Battery Systems
Some scooter battery packs have their own battery management board (BMS). The bridge can read battery info from:
- The VESC itself
- The Ninebot Original BMS
- A JBD (Xiaoxiang) BMS

### Supports Two Board Versions
There are two hardware versions of the board (V1.0 and V2.1+). You pick which one you have in the settings page and the device automatically uses the right pins.

### Tracks Multiple Motor Controllers
If your scooter has more than one VESC (e.g. dual motors), the bridge reads speed data from all of them and averages the values together for a smooth, accurate speed reading.

## Quick Summary Table

| Feature | What it means |
|---|---|
| X3 Series Support | Supports G3, ZT3 and GT3* |
| Speed display | Shows km/h on the dashboard |
| Battery display | Shows battery % on the dashboard |
| Turn signals | Flashes left/right lights |
| Rear light | Controls the back light |
| Horn | Support for the GT3 horn |
| Ride profiles | Switches power modes from the dashboard |
| Low battery lock | Blocks the dashboard profile application when the battery is low |
| Throttle & brake | VESC reads the throttle and brake directly from the Dashboard |
| Wi-Fi OTA updates | Update firmware from a browser |
| Web config page | Change settings from a browser |
| Multiple BMS support | Works with VESC, Ninebot, or JBD battery boards |
| Two board versions | Supports V1.0 and V2.1+ hardware |
| Multi-VESC support | Handles more than one motor controller |

*This device was originally developed for the G3
---

## How to create a PCB from the files

1. Clone this repository and open it with KiCad.  
2. Use the **"Fabrication Outputs"** option to generate the required files for your PCB manufacturer.

<img src="https://media.discordapp.net/attachments/1484589733932830810/1484589753570693221/image.png?ex=69bec773&is=69bd75f3&hm=8f9db0c190a1125b79901996c782525d298798a685708147448ac20ca4147bc0&=&format=webp&quality=lossless" alt="PCB preview">

3. Upload the files to a PCB manufacturer (for example, JLCPCB) and choose a 2-layer PCB board.  
4. Once you receive your PCB, development can begin. :)

## Firmware flashing

1. The board uses an **ESP32-S3-WROOM-1** chip, which has exposed USB D+, D-, and GND pins on the PCB.  
2. Use these pins to connect a USB cable to your computer and flash your firmware.

## Flashing Service

We offer a PCB flashing service for those who prefer a ready-to-use board:  

- **Cost:** Listed on our [shop page](https://scooter-labs.com/shop)  
- **Includes:** Lifetime free firmware updates  
- **How it works:** Simply order the service, and we will program your PCB with the latest firmware and ship it to you ready to use.  

This service is optional — you can still build and flash the PCB yourself if you prefer.

---

# License
This PCB design is released under the [CC BY-NC-SA 4.0 License](https://creativecommons.org/licenses/by-nc-sa/4.0/).

## What you can do
- Share and adapt this design for personal or hobby purposes
- Give credit to the original author (Finn Tews)

## What you cannot do
- Use this design for commercial purposes
- Remove or obscure attribution
- Re-license derivatives under more restrictive terms