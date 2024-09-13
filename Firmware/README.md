# BlackPearl

## Conectare placa la baterie
## Folosire baterii

### Incarcare baterii

#### Componente
- **Incarcator**: DFR0564
- **Baterii**: 7.4V lithium

#### Instructiuni de incarcare

1. **Conectare**: Conectati bateria la sistemul de incarcare utilizand firele cu conectori tata-mama.
2. **Monitorizare incarcare**:
	- Daca LED-ul **CHG** de pe incarcator este aprins constant, inseamna ca bateria se incarca.
	- Cand bateria este complet incarcata, LED-ul **CHG** se va stringe sau va incepe sa clipeasca rar.
	- Daca LED-ul **CHG** clipeste rapid, acest lucru indica faptul ca bateria nu este conectata corect la sistemul de incarcare.

### Conectare placa la baterie

- Conectati bateria la circuit utilizand firele cu conector tata. Firul negru trebuie să fie conectat la pinul **GND**.

### Referințe:
- [Documentația încărcătorului DFR0564](https://wiki.dfrobot.com/USB_Charger_for_7.4V_LiPo_Battery_SKU__DFR0564#target_3)
- [Tutorial video pentru utilizarea încărcătorului](https://www.youtube.com/watch?v=iOwl5zBeYW0)
- [Tutorial video pentru conectarea bateriei la circuit](https://www.youtube.com/shorts/zBXLMM8SL_8)



## Design rotor cu sfoara pentru ridicat barca
### Circuit initial

![Motor Circuit](https://github.com/ABC-practica-2024/BlackPearl/blob/Design-basic-pentru-coborat/ridicat-camera/Firmware/Motor%20Circuit.png)
#### Componente circuit
- **Arduino**
- **Motor DC**
- **Baterie**
- **Motor Driver**

#### Design
  - De motor era atasat un ax pe care se afla un cablu, in functie de directia de rotatie a motorului, cablul era intins sau tras, la capatul cablului este atasata cutia.


### Circuit Secundar
![Brushless Motor Circuit](https://github.com/ABC-practica-2024/BlackPearl/blob/Design-basic-pentru-coborat/ridicat-camera/Firmware/image.png)
#### Componente circuit
- **Brushless ESC S-25A**
- **Battery**
- **4Poles Motor**
- **Arduino**

### Componente
- **Mosor/Bobina**
- **Ata de pescuit**
- **Ax si motor melcat**  
### Design
- Motorul va invarti un ax melcat, care va actiona o roata melcata de care va fi atasat un mosor/o bobina pe care va fi infasurata ata de pescuit. Motorul va fi actionat de arduino, avand cod pentru push, pull si stop.
![Design rotor](https://github.com/ABC-practica-2024/BlackPearl/blob/Design-basic-pentru-coborat/ridicat-camera/Firmware/Design%20Rotor.png)

### Referinte
-[Brushless ESC Motor](https://www.youtube.com/watch?v=qOzE5F5vFGs)
-[Controlling ESC Motor](https://www.youtube.com/watch?v=-EjVWE8KvKE)
-[Beeping error and Motor calibration](https://forum.flitetest.com/index.php?threads/esc-calibration-and-fast-beeping.8449/)
-[Manual Sensorless Brushless](https://www.himodel.com/en/info/manual/ESC_Manual_english_Himodel_20060625.pdf)
-[Sending data over MQTT](https://docs.arduino.cc/tutorials/uno-wifi-rev2/uno-wifi-r2-mqtt-device-to-device/)
-[Arduino DC Motor](https://www.tutorialspoint.com/arduino/arduino_dc_motor.htm)
