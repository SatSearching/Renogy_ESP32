# Renogy_ESP32
Little project to read data from a Renogy Solar Controller and display on a website

Credit to the below as this project is based on it.

https://github.com/wrybread/ESP32ArduinoRenogy

At the bottom of my garden, I have a small steel shed with 2 solar panels on the roof and a Renogy Rover 40A MPTT Charge Controller inside.
Also fitted to my fence are LED lights, I wanted a method to control the lights and read data from within the charge controller via the RS232 RJ12 port
so that I could work out if I needed more panels or batteries, I found the available BT options was a bit limited.

I found the above project by wrybread" and decided to work from that, but using an ESP32 so that data can be displayed on my mobile phone via an internal site as below.

<img width="432" height="904" alt="image" src="https://github.com/user-attachments/assets/aac7cb14-3e40-4d1b-9b07-34ceaccc1a4f" />

This readings backgrounds are dynamic meaning they change colour depending on state. I have also used 2 PWM boards so I can change brightness etc etc.

One I got this up and running, a friend wanted to do the same and bought the same charge controller, but found these charge controllers now come with an RS485 RJ45 port.
He left his controller with me for a couple of days and I managed to change my code so that it works on RS232 and RS485.

For RS232, you need a cheap RS232 to TTL converter, for RS485, you need a cheap RS485 to TTL converter, my code works on either without having to select anything.

RS485 Converter as used.
![max485](https://github.com/user-attachments/assets/17d5034b-eed4-47a5-bc85-863e90c25a85)

RS232 Converter as used.
<img width="207" height="122" alt="image" src="https://github.com/user-attachments/assets/8201b51f-2daa-4913-85fe-167d25d12d03" />


Two circuit diagrams included of my system showing the different connections.

My code is probably messy but works, I have added a lot of comments to help others that may be interested.
Ensure you edit your WIFI ID and password, also your prefered IP details.

This project and software is provided “as is," and you use at your own risk!

![ESP32-S3 wif FOR DRAWINGSi](https://github.com/user-attachments/assets/7306e0a4-803c-46fd-acbf-725fe252dd06)




