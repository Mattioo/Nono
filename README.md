# Nono
<br />

### Części
* Nazwa: 4WD Chassis - czterokołowe podwozie robota Dagu
* Cena: 109,00 zł
* Link: https://botland.com.pl/podwozia-robotow/2537-4wd-chassis-czterokolowe-podwozie-robota-dagu-.html
<br />

![4wd-chassis-czterokolowe-podwozie-robota-dagu-](https://user-images.githubusercontent.com/11798406/31551813-fa370fc2-b035-11e7-9f36-f6e4c9a5acac.jpg)

* Nazwa: Silnik elektryczny Motraxx X-Slot Tuning, skala 1:24 (x4)
* Cena: 66,16 zł
* Link: https://www.conrad.pl/p/silnik-elektryczny-motraxx-x-slot-tuning-skala-124-229020?utm_medium=Email&utm_source=Order_confirmation&utm_term=229020
<br />

![image](https://user-images.githubusercontent.com/11798406/36328475-8b30ae9a-1362-11e8-8a23-f0a8626a8489.jpg)

* Nazwa: Moduł WiFi ESP8266 + NodeMCU v3
* Cena: 39,90 zł
* Link: https://botland.com.pl/moduly-wifi/8241-modul-wifi-esp8266-nodemcu-v3.html?search_query=esp8266&results=55
<br />

![capture](https://user-images.githubusercontent.com/11798406/36346576-abb753f6-1440-11e8-991f-7ebd8a4e7ee2.PNG)

* Nazwa: Akumulator SAMSUNG ICR 18650-22P (x4)
* Cena: 48,00 zł
* Link: https://www.tme.eu/pl/details/accu-18650-2.2p-hv/akumulatory/samsung/icr-18650-22p/
<br />

![9930](https://user-images.githubusercontent.com/11798406/36340414-adf7f224-13dc-11e8-97c8-e4cd1ee7b84f.jpg)

* Nazwa: Pojemnik COMF BHC-18650-1P (x4)
* Cena: 12,52 zł
* Link: https://www.tme.eu/pl/details/bhc-18650-1p/baterie-pojemniki-i-uchwyty/comf/
<br />

![bat](https://user-images.githubusercontent.com/11798406/36340438-3216b496-13dd-11e8-84f3-09c9b23e935c.jpg)

* Nazwa: L298N - dwukanałowy sterownik silników - moduł WB291111
* Cena: 24,90 zł
* Link: https://botland.com.pl/sterowniki-silnikow-moduly/8227-l298n-dwukanalowy-sterownik-silnikow-modul-wb291111.html?search_query=%09L298+-+dwukanalowy+sterownik+silnikow+-+modul&results=10
<br />

![l298n-dwukanalowy-sterownik-silnikow-modul-wb291111](https://user-images.githubusercontent.com/11798406/36329907-f3a82cfa-1367-11e8-8d12-70b9a4fa4ef2.jpg)

* Nazwa: Serwo TowerPro SG-90 - micro (x2)
* Cena: 12,90 zł
* Link: https://botland.com.pl/serwa-tower-pro/484-serwo-towerpro-sg-90-micro.html?search_query=Serwo+TowerPro+SG-90+-+micro&results=29
<br />

![serwo-towerpro-sg-90-micro](https://user-images.githubusercontent.com/11798406/31552113-cc20b330-b036-11e7-88e6-10eea36a7440.jpg)

* Nazwa: Uchwyt do serw micro Pan/Tilt z miejscem na kamerę - plastikowy
* Cena: 13,90 zł
* Link: https://botland.com.pl/chwytaki-uchwyty-gimbale/3724-uchwyt-do-serw-micro-pantilt-z-miejscem-na-kamere-plastikowy.html?search_query=Uchwyt+do+serw+micro+Pan%2FTilt+z+miejscem+na+kamere+-+plastikowy&results=1
<br />

![uchwyt-do-serw-micro-pantilt-z-miejscem-na-kamere-plastikowy](https://user-images.githubusercontent.com/11798406/31552170-f52798c0-b036-11e7-863e-ad882768a3cd.jpg)

* Nazwa: Przewody połączeniowe żeńsko-męskie 10cm - 40szt.
* Cena: 7,00 zł
* Link: https://botland.com.pl/przewody-polaczeniowe/6174-przewody-polaczeniowe-zensko-meskie-10cm-40szt.html?search_query=Przewody+polaczeniowe+&results=130
<br />

![przewody-polaczeniowe-zensko-meskie-10cm-40szt](https://user-images.githubusercontent.com/11798406/31552302-5df4fcbc-b037-11e7-8c5d-263d8bae4db9.jpg)

### Połączenia przewodów

![download](https://user-images.githubusercontent.com/11798406/36346580-c54d32fe-1440-11e8-8d02-fcc10ce7f97c.png)

using RestSharp;
using RestSharp.Authenticators;
using System;
using System.IO;
using System.Text;

namespace mailer
{
    class Program
    {
        static void Main(string[] args)
        {
            try
            {
                if (args.Length < 4)
                    throw new Exception("Nie podano wszystkich parametrów. Sprawdź czy wprowadzono: <email> <tytul> <plik.txt> <timeout (s)>");

                string address = args[0];
                string title = args[1];
                string file = args[2];
                int timeout = int.Parse(args[3]);

                using (StreamReader sr = new StreamReader(file))
                {
                    String text = File.ReadAllText(file, Encoding.UTF8);
                    while (true)
                    {
                        SendSimpleMessage(address, title, text);
                        System.Threading.Thread.Sleep(timeout * 1000);
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
        }

        public static IRestResponse SendSimpleMessage(string address, string title, string text)
        {
            var client = new RestClient
            {
                BaseUrl = new Uri("https://api.mailgun.net/v3"),
                Authenticator =
                new HttpBasicAuthenticator("api", "a2811ac192efa34b91eaf6ffa73aad2f-52cbfb43-7fd03e6f")
            };

            var request = new RestRequest();
            request.AddParameter("domain", "sandbox0b7ddebaa735417988ad64cb618cee2a.mailgun.org", ParameterType.UrlSegment);
            request.Resource = "{domain}/messages";
            request.AddParameter("from", "Mailgun Sandbox <postmaster@sandbox0b7ddebaa735417988ad64cb618cee2a.mailgun.org>");
            request.AddParameter("to", address);
            request.AddParameter("subject", $"{title}_{Guid.NewGuid()}");
            request.AddParameter("text", text);
            request.Method = Method.POST;
            return client.Execute(request);
        }
    }
}

