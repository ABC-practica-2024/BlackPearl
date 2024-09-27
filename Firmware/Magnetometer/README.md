# Documentație pentru QMC5883L: Unghiul Azimuth

Acest proiect utilizeaza senzorul QMC5883L  pentru a calcula unghiul **Azimuth** si il afiseaza pe monitorul serial.

## 1. Configurarea senzorului QMC5883L 

Pentru a obrine date precise, senzorul QMC5883L  este configurat manual prin accesarea registrilor interni folosind protocolul I2C:
- **Mod de operare**: Mod continuu
- **Rata de date de ieșire**: 10 Hz
- **Interval de scalare**: +-8g
- **Supraexemplarizare**: OSR 512


QMC5883L este setat pentru a citi datele brute și pentru a le trimite mai departe pentru procesare.

## 2. Calibrarea magnetometrului

Pentru a obține rezultate precise, se efectuează calibrarea soft și hard iron a senzorului QMC5883L. Acest proces implică:
- **Calibrarea hard iron**: Aceasta elimină biasul de offset generat de sursele de câmp magnetic din jur. Se măsoară offset-ul pe fiecare axă și se aplică aceste corecții.
- **Calibrarea soft iron**: Aceasta corectează distorsiunile cauzate de obiectele metalice din apropierea senzorului. Se determină o matrice de corecție care se aplică datelor dupa aplicarea calibrarii hard iron.

### Obtinerea datelor de calibrare

- **Colectarea datelor brute**: Rotirea magnetometrului in jurul a 14 axe diferite pentru a acoperi campul magnetic din cat mai multe pozitii.
- **Utilizarea Magneto12 pentru calibrare**: Datele colectate au fost ulterior introduse în aplicația Magneto12, un program specializat în calcularea coeficienților de calibrare. Acest software utilizează un algoritm matematic pentru a determina matricile de corecție pentru soft iron și hard iron, necesare pentru a compensa distorsiunile și offset-ul cauzate de mediul înconjurător.



## 3. Citirea și prelucrarea datelor oferite de magnetometru

Magnetometrul măsoară câmpul magnetic pe axele X, Y, și Z. Folosim aceste date pentru a calcula unghiul **Azimuth** în raport cu orientarea sa față de Nordul magnetic. Unghiul este obținut folosind datele corectate de calibrare in funcția `atan2()` .

## 4. Referințe utile

##Referinte

- [Video tutorial](https://www.youtube.com/watch?v=RJovRafwgo8)
