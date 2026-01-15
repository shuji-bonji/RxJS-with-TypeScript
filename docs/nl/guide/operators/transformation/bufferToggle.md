---
description: De bufferToggle operator is een geavanceerde bufferoperator die start- en eindtriggers laat controleren door aparte Observables en meerdere bufferperiodes onafhankelijk laat beheren.
titleTemplate: ':title'
---

# bufferToggle - Start/Einde Controle

De `bufferToggle` operator controleert de **starttrigger** en **eindtrigger** met aparte Observables en geeft de waarden uit in een array. Dit is een geavanceerde bufferoperator die meerdere bufferperiodes tegelijk kan beheren.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // Geef waarden elke 0,5 seconden uit

// Starttrigger: elke 2 seconden
const opening$ = interval(2000);

// Eindtrigger: 1 seconde na start
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output:
// [3, 4, 5]     (Start op 2 sec, eindigt op 3 sec)
// [7, 8, 9]     (Start op 4 sec, eindigt op 5 sec)
// [11, 12, 13]  (Start op 6 sec, eindigt op 7 sec)
```

**Werkingsstroom**:
1. `opening$` zendt een waarde uit → Buffering start
2. Observable geretourneerd door `closing()` zendt een waarde uit → Buffering eindigt, voert array uit
3. Meerdere bufferperiodes kunnen overlappen

[🌐 RxJS Officiële Documentatie - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)

## 🆚 Contrast met andere buffer operators

`bufferToggle` is uniek vergeleken met andere buffer operators doordat het **onafhankelijke controle** van start en einde toestaat.

### Vergelijking van elke operator

| Operator | Trigger | Kenmerk | Gebruiksscenario |
|---|---|---|---|
| `buffer(trigger$)` | Enkele Observable | Eenvoudig | Event-gedreven buffering |
| `bufferTime(ms)` | Tijd | Periodiek | Data-aggregatie op regelmatige intervallen |
| `bufferCount(n)` | Aantal | Kwantitatief | Verwerking in eenheden van N |
| `bufferToggle(open$, close)` | Aparte start/eind controle | Flexibel | Complexe periodebeheer |

### Codevoorbeeld vergelijking

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // Geef 0-9 elke 300ms uit

// bufferToggle: Onafhankelijke controle van start en einde
const opening$ = interval(1000); // Start elke 1 seconde
const closing = () => interval(500); // Eindig 500ms na start

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4], [6, 7], [9]
//
// Tijdlijn:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [Start       Eind]   [Start       Eind]   [Start Eind]
//                  └→ [3,4]            └→ [6,7]            └→ [9]
```

**Gebruiksrichtlijnen**:
- **`buffer`** → Voer buffer uit elke keer dat trigger Observable een waarde uitzendt
- **`bufferTime`** → Voer buffer automatisch uit op regelmatige intervallen
- **`bufferCount`** → Voer buffer uit wanneer gespecificeerd aantal is bereikt
- **`bufferToggle`** → Aparte start/eind controle, overlappende periodes mogelijk

> [!TIP]
> Voor meer details over elke operator, zie [buffer](/nl/guide/operators/transformation/buffer), [bufferTime](/nl/guide/operators/transformation/bufferTime), [bufferCount](/nl/guide/operators/transformation/bufferCount).

## 💡 Typische gebruikspatronen

1. **Dataverzameling tijdens kantooruren**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Sensordata (altijd verzamelend)
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // Kantoor open: 9:00 (Simulatie: na 2 seconden)
   const businessOpen$ = timer(2000, 10000); // Na 2 sec, dan elke 10 sec

   // Kantoor sluit: 5 seconden na start
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`Data tijdens kantooruren: ${data.length} items`);
     console.log(`Gemiddelde: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **Gebeurtenisregistratie tijdens knop indrukken**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Vasthouden';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // Datastroom
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // Start: Muis omlaag
   const mouseDown$ = fromEvent(button, 'mousedown');

   // Einde: Muis omhoog (van mousedown naar mouseup)
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `Gebeurtenissen geregistreerd tijdens vasthouden: ${events.length} items`;
     console.log('Geregistreerde gebeurtenissen:', events);
   });
   ```

## 🧠 Praktisch codevoorbeeld (Downloadperiodebeheer)

Dit is een voorbeeld van het beheren van datadownloadperiodes met start- en stopknoppen.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// Maak UI-elementen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Datadownloadbeheer';
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = 'Start';
container.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Wachten...';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Datastroom (genereer downloaddata elke 1 seconde)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Start- en stoptriggers
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Gestopt';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Downloaden...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Buffering
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download voltooid</strong><br>
    Aantal: ${downloads.length} items<br>
    Totale grootte: ${(totalSize / 1024).toFixed(2)} MB<br>
    Gemiddelde grootte: ${avgSize.toFixed(0)} KB
  `;

  console.log('Downloaddata:', downloads);
});
```

## 🎯 Overlappende bufferperiodes

Een kenmerk van `bufferToggle` is dat het meerdere bufferperiodes tegelijk kan beheren.

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Start: elke 1 seconde
const opening$ = interval(1000);

// Einde: 1,5 seconden na start
const closing = () => interval(1500);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output:
// [4, 5, 6]        (Start op 1 sec → Eindigt op 2,5 sec)
// [9, 10, 11, 12]  (Start op 2 sec → Eindigt op 3,5 sec) ※Gedeeltelijke overlap
// [14, 15, 16, 17] (Start op 3 sec → Eindigt op 4,5 sec)
```

**Tijdlijn**:
```
Bron:      0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Start:     ----1sec----2sec----3sec----4sec
Periode 1: [------1.5sec-----]
            └→ Uitvoer: [4,5,6]
Periode 2:        [------1.5sec-----]
                   └→ Uitvoer: [9,10,11,12]
Periode 3:               [------1.5sec-----]
                          └→ Uitvoer: [14,15,16,17]
```

## ⚠️ Veelgemaakte fouten

> [!WARNING]
> `bufferToggle` kan meerdere bufferperiodes tegelijk beheren, maar als starttriggers te frequent vuren, zullen er veel buffers tegelijk bestaan, wat geheugen verbruikt.

### Fout: Starttriggers te frequent

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ❌ Slecht voorbeeld: Start elke 100ms, eindig na 5 seconden
const opening$ = interval(100); // Te frequent
const closing = () => interval(5000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Tot 50 buffers kunnen tegelijk bestaan → Geheugenrisico
```

### Correct: Stel passend interval in

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ✅ Goed voorbeeld: Stel passend interval voor start in
const opening$ = interval(2000); // Elke 2 seconden
const closing = () => interval(1000); // Buffer voor 1 seconde

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Maximaal 1-2 buffers bestaan tegelijk
```

## 🎓 Samenvatting

### Wanneer bufferToggle gebruiken
- ✅ Wanneer u start en einde onafhankelijk wilt controleren
- ✅ Wanneer u data wilt verzamelen voor een beperkte periode, zoals tijdens knop indrukken
- ✅ Wanneer u meerdere bufferperiodes tegelijk wilt beheren
- ✅ Dataverzameling onder complexe voorwaarden, zoals alleen tijdens kantooruren

### Wanneer buffer/bufferTime/bufferCount gebruiken
- ✅ Wanneer eenvoudige periodieke buffering voldoende is
- ✅ Wanneer een enkele trigger voldoende is voor controle

### Wanneer bufferWhen gebruiken
- ✅ Wanneer alleen de eindvoorwaarde dynamisch moet worden gecontroleerd
- ✅ Wanneer continue bufferperiodes nodig zijn

### Opmerkingen
- ⚠️ Frequente starttriggers veroorzaken dat veel buffers tegelijk bestaan, wat geheugen verbruikt
- ⚠️ Bufferperiodes kunnen overlappen
- ⚠️ Kan moeilijk te debuggen zijn vanwege complexe controles

## 🚀 Volgende stappen

- [buffer](/nl/guide/operators/transformation/buffer) - Leer basis buffering
- [bufferTime](/nl/guide/operators/transformation/bufferTime) - Leer tijd-gebaseerde buffering
- [bufferCount](/nl/guide/operators/transformation/bufferCount) - Leer aantal-gebaseerde buffering
- [bufferWhen](https://rxjs.dev/api/operators/bufferWhen) - Leer dynamische eindcontrole (officiële documentatie)
- [Transformatieoperator praktische use cases](/nl/guide/operators/transformation/practical-use-cases) - Leer echte use cases
