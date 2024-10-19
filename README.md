# 1. Wprowadzenie
## 1.1 Cel projektu
## 1.2 Konfiguracja

* Zrootować gogle oraz air unit wykorzystując platformę WTFOS (https://fpv.wtf/) i zainstalować przez dostępny tam PackageManager pakiet msp-osd (dostępny również w repozytorium: [msp-osd-0.10.1.zip](External/msp-osd-0.10.1.zip)) na obu urządzeniach umożliwiając interpretacje komunikatów protokołu komunikacyjnego MSP.
* Włączyć w ustawieniach gogli opcję "Custom OSD".
* Doinstalować do Arduino IDE bibliotekę ReefwingMSP v. 2.0.0 (dostępną również w repozytorium: [Reefwing-MSP-main.zip](External/Reefwing-MSP-main.zip)).
* Skonfigurować Arduino IDE do pracy z Arduino Due:  
Tools -> Board -> Arduino SAM Boards(32-bits ARM Cortex-M3) -> Arduino Due (Programming Port).

# 2. Komponenty
## 2.1. Elementy konstrukcyjne

* Kadłub: WT-200 (złożona metalowa obudowa z silnikami, gąsienicami i wbudowanym oświetleniem bez elektroniki) ~2000 zł uwzględniając podatek  
https://www.ebay.com/itm/355056959871?norover=1&mkevt=1&mkrid=711-169407-669907-5&mkcid=2&itemid=355056959871&targetid=296739514498&device=c&mktype=pla&googleloc=1011598&poi=&campaignid=21082369877&mkgroupid=160701766018&rlsatarget=pla-296739514498&abcId=&merchantid=5399845611&gad_source=1&gclid=Cj0KCQjwsc24BhDPARIsAFXqAB3d4eODbke1557Q6mTR9yhPd24dqa3JCsqlJvveJ4PRz2DknQPjAOgaArrIEALw_wcB

* Uchwyt do serw: standard Pan/Tilt (x3) ~ 66 zł  
https://botland.com.pl/uchwyty-i-mocowania-do-serw/13500-uchwyt-do-serw-standard-pantilt-5904422358686.html

* Aluminiowy okrągły orczyk - 20mm / 6mm (x3) ~ 9 zł  
https://botland.com.pl/orczyki-do-serw/12538-aluminiowy-okragly-orczyk-20mm-6mm-5904422319533.html

* Górna część obudowy zaprojektowana w AutoCAD 2024 i wycięta laserem przez firmę AJMaker na podstawie przygotowanego projektu [Nono.dxf](External/Nono.dxf) ~ 350 zł  
https://ajmaker.pl/produkt/ciecie-laserem/

## 2.2. Elementy elektroniczne

* Wentylator 08025DA-12P-AA-00 [80x80x25] 12VDC (chłodzenie air unit - 75 CFM) ~ 110zł  
https://www.mouser.pl/ProductDetail/NMB-Technologies/08025DA-12P-AA-00?qs=byeeYqUIh0PE89S%2FBCp52A%3D%3D

*  Serwo Feetech FT6335M - standard (x3) ~ 330 zł  
https://botland.com.pl/serwa-typu-standard/17694-serwo-feetech-ft6335m-standard-5904422328474.html

* Buzzer z zasilaniem LiPo Iflight YR50B_S Finder Buzzer 100dB ~ 50 zł  
https://dronexpert.eu/buzzer-z-zasilaniem-lipo-iflight-yr50b-s-finder-buzzer-100db-p-14827.html

* Przetwornica Buck 3A wielokanałowa 3.3V 5V 12V ~ 35 zł  
https://allegro.pl/oferta/przetwornica-buck-3a-wielokanalowa-3-3v-5v-12v-14156675359

* Moduł przekaźnika Iduino - 1 kanał - styki 10A/250VAC - cewka 5V ~ 7 zł  
https://botland.com.pl/przekazniki-przekazniki-arduino/8228-modul-przekaznika-iduino-1-kanal-styki-10a250vac-cewka-5v-5903351241229.html?cd=18298825651&ad=&kd=&gad_source=1&gclid=Cj0KCQjwsc24BhDPARIsAFXqAB3QbrJrIXz3Gwd0XWCuOFroiZ-3YjjVR135QzmRXGMxBxCA4fUP6KsaAnkHEALw_wcB

* Czujnik napięcia 0-25V dzielnik 5V/25V ~ 4 zł  
https://allegro.pl/oferta/770-czujnik-napiecia-0-25v-dzielnik-5v-25v-arduino-13720748764

* Ultradźwiękowy czujnik odległości A02YYUW 3-450cm - wodoodporny - DFRobot SEN0311 ~ 105 zł  
https://botland.com.pl/ultradzwiekowe-czujniki-odleglosci/15717-ultradzwiekowy-czujnik-odleglosci-a02yyuw-3-450cm-wodoodporny-dfrobot-sen0311-5904422377984.html?cd=20567593583&ad=&kd=&gad_source=1&gclid=Cj0KCQjwsc24BhDPARIsAFXqAB2PdXAQ75-6Ny77AunFOs1t5L1gmo8vVB5554h2xEbLI8bjrPAJ4aUaAk5XEALw_wcB

* DFRobot BTS7960 - dwukanałowy sterownik silników 35V/15A ~ 260 zł  
https://botland.com.pl/produkty-wycofane/2691-dfrobot-bts7960-dwukanalowy-sterownik-silnikow-35v-15a-5903351243001.html

* Cytron SC08A - sterownik serw 8-kanałowy - UART ~ 72 zł  
https://botland.com.pl/sterowniki-serw/12396-cytron-sc08a-sterownik-serw-8-kanalowy-uart-5904422319137.html

* Arduino Due ARM Cortex ~ 215 zł  
https://botland.com.pl/arduino-seria-podstawowa-oryginalne-plytki/1214-arduino-due-arm-cortex-a000062-7630049200487.html?cd=18298825651&ad=&kd=&gad_source=1&gclid=Cj0KCQjwsc24BhDPARIsAFXqAB0eY-vECM-w-QgRXqhN7smcFOxYbvRtrhOl6RkTT_-UTjtrCfVOldIaAsOKEALw_wcB

* Akumulator 6500mAh 11.1V 60C BASHING Gens Ace LiPo (x2) ~ 700 zł  
https://allegro.pl/oferta/akumulator-6500mah-11-1v-60c-bashing-gens-ace-lipo-14795865034?utm_feed=aa34192d-eee2-4419-9a9a-de66b9dfae24&utm_source=google&utm_medium=cpc&utm_campaign=_elktrk_rtvagd_pla_pmax_bpg&ev_campaign_id=21823047314&gad_source=1&gclid=Cj0KCQjwsc24BhDPARIsAFXqAB0Kw_ePvgkdK10tTz_M3XlffZMfObGihc1DgXJ3vKlefNe4oD-2gUMaAnlSEALw_wcB

* Nadajnik z kamerą Caddx Air Unit Polar starlight Digital HD DJI FPV system	~ 940 zł  
https://www.nobshop.pl/nadajnik-z-kamera-caddx-air-unit-polar-starlight-digital-hd-dji-fpv-system-p-2978.html

## 2.3. Elementy wyposażenia

* Gogle DJI FPV V2 ~ 3000 zł  
https://ironsky.pl/drony-sklep/produkt/dji-fpv-goggles-v2/?srsltid=AfmBOopl98Kduze9ftT5VyNm-hO431iGa1I_OtloEMRCiHwcjjEIiese

* Kontroler TBS Tango 2 (Team Blackspeep) ~ 900 zł  
https://www.ebay.com/itm/145920613727?_skw=team+blacksheep+tango+2+pro&itmmeta=01JAJEEMV89AQYB96F29622V1W&hash=item21f98bdd5f:g:KNgAAOSwk5tmqT6B&itmprp=enc%3AAQAJAAAA4HoV3kP08IDx%2BKZ9MfhVJKllJQmODqyFHTJrScGLve1TH%2F76BQsyXDW2tROL4CbuuxmPWG%2FIgGwpQvXbXdwWJ7ZUy26Tw4M6vKz7KCb8zXnN%2BlRFyxq3MwnzkDIhFXaT4FNHJ0eb5jHBGkLlR%2FXv6%2FS9NL9lpteB0Xnlyi%2F9TYXIW1IB9LhddS%2FY64iULPoAfydMg1iw7yB6cBS3tOTW0xdhjt3tl%2F11%2F5JA%2FfOOBIj7cnLg7sAB8UWMBwC918y76qh%2BiNJclxqtsCIpADwhjbWt96gmoXMg6CicdOe%2BR3hr%7Ctkp%3ABk9SR97Nus7UZA

## 2.4. Dodatkowe elementy

* 181 Radiator aluminiowy 9x9x5mm (x10) ~ 17 zł  
https://allegro.pl/oferta/181-radiator-aluminiowy-9x9x5mm-9382407076

* M4 NAKRĘTKA KWADRATOWA NISKA 562 OCYNKOWANA 10SZT (x2) ~ 4 zł  
https://allegro.pl/oferta/m4-nakretka-kwadratowa-niska-562-ocynkowana-10szt-13056354425

* M4x30 ŚRUBA SZEŚCIOKĄTNA NIERDZEWNA A2 933 (10) (x2) ~ 5 zł  
https://allegro.pl/oferta/m4x30-sruba-szesciokatna-nierdzewna-a2-933-10-14255374599

* Podkładki płaskie 720 sztuk stal nierdzewna M2-M12 ~ 45 zł  
https://allegro.pl/oferta/podkladki-plaskie-720-sztuk-stal-nierdzewna-m2-m12-12966456002?bi_s=ads&bi_m=productlisting:desktop:query&bi_c=YWY1NjQ1OTctMDc4ZC00Mzk1LWIyOTEtY2M4Njk1ODJjOWRiAA&bi_t=ape&referrer=proxy&emission_unit_id=4bfd0f20-64d3-42c8-ba4c-36c5e7be9fc2

* ZESTAW 440 ŚRUB 6-KĄTNE HEX ZE STALI NIERDZEWNEJ ~ 27 zł  
https://allegro.pl/oferta/zestaw-440-srub-6-katne-hex-ze-stali-nierdzewnej-16119291441?bi_s=ads&bi_m=productlisting:desktop:query&bi_c=ZGQxYTYyNzAtYTIyZC00YjA4LWE1NzMtMmZhZTY0MzA1NmJkAA&bi_t=ape&referrer=proxy&emission_unit_id=ae0418b0-7877-4451-9b7d-ad68f606ec56

* Zestaw nylonowych śrubek i podkładek dystansowych M3 - 180 elementów ~ 34 zł  
https://botland.com.pl/produkty-wycofane/13577-zestaw-nylonowych-srubek-i-podkladek-dystansowych-m3-180-elementow-5904422320973.html

* Przedłużacz do serw 45cm (x3) ~ 12 zł  
https://botland.com.pl/przewody-do-serw/1387-przedluzacz-do-serw-45cm-5904422350857.html

* Przewody połączeniowe żeńsko-męskie 30cm kolorowe - 50szt. ~ 30 zł  
https://botland.com.pl/przewody-polaczeniowe-zensko-meskie/1438-przewody-polaczeniowe-zensko-meskie-30cm-kolorowe-50szt-5904422349400.html

* Przewody połączeniowe męsko-męskie 30cm kolorowe - 50szt. ~ 28 zł  
https://botland.com.pl/przewody-polaczeniowe-mesko-meskie/1437-przewody-polaczeniowe-mesko-meskie-30cm-kolorowe-50szt-5904422356002.html

* XT90H komplet z osłonkami, konektor AMASS oryginał (x2) ~ 16 zł  
https://allegro.pl/oferta/xt90h-komplet-z-oslonkami-konektor-amass-oryginal-8186134922

# Schemat połączeń
..
