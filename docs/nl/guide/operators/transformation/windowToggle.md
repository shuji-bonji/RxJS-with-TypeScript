---
description: windowToggle is een geavanceerde RxJS conversieoperator die meerdere vensterperiodes onafhankelijk laat beheren, met start- en eindtriggers gecontroleerd door aparte Observables. Het is ideaal voor situaties waar dynamisch periodebeheer vereist is, zoals dataverzameling tijdens kantooruren of gebeurtenisregistratie tijdens knop indrukken. TypeScript type-inferentie zorgt voor type-veilige venstersplitsingsverwerking.
titleTemplate: ':title | RxJS'
---

# windowToggle - Venster met onafhankelijke start- en eindcontrole

De `windowToggle` operator controleert **starttrigger** en **eindtrigger** met aparte Observables, waarbij elke periode als een nieuwe Observable wordt uitgegeven. Dit is een geavanceerde vensteroperator die meerdere vensterperiodes tegelijk kan beheren.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // Geef waarden elke 0,5 seconden uit

// Starttrigger: elke 2 seconden
const opening$ = interval(2000);

// Eindtrigger: 1 seconde na start
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Waarde in venster:', value);
});

// Start op 2 sec, eindigt op 3 sec → Waarden: 4, 5
// Start op 4 sec, eindigt op 5 sec → Waarden: 8, 9
// Start op 6 sec, eindigt op 7 sec → Waarden: 12, 13
```

**Werkingsstroom**:
1. `opening$` zendt een waarde uit → Venster start
2. Observable geretourneerd door `closing()` zendt een waarde uit → Venster eindigt
3. Meerdere vensterperiodes kunnen overlappen

[🌐 RxJS Officiële Documentatie - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 Typische gebruikspatronen

- Dataverzameling tijdens kantooruren
- Gebeurtenisregistratie tijdens knop indrukken
- Actietracking tijdens actieve sessies
- Streamverwerking die dynamisch periodebeheer vereist

## 🔍 Verschil met bufferToggle

| Operator | Uitvoer | Gebruiksscenario |
|:---|:---|:---|
| `bufferToggle` | **Array (T[])** | Gegroepeerde waarden samen verwerken |
| `windowToggle` | **Observable&lt;T&gt;** | Verschillende streamverwerking voor elke groep |

```ts
import { interval } from 'rxjs';
import { bufferToggle, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500);
const opening$ = interval(2000);
const closing = () => interval(1000);

// bufferToggle - Uitvoer als array
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Uitvoer: Buffer (array): [4, 5]
});

// windowToggle - Uitvoer als Observable
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  console.log('Venster (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Waarde in venster:', value);
  });
});
```

## 🧠 Praktisch codevoorbeeld 1: Gebeurtenissen registreren tijdens knop indrukken

Dit is een voorbeeld van het registreren van data tussen muis omlaag en muis omhoog.

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// Maak knop
const button = document.createElement('button');
button.textContent = 'Vasthouden';
document.body.appendChild(button);

// Uitvoergebied
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// Datastroom (elke 100ms)
const data$ = interval(100);

// Start: Muis omlaag
const mouseDown$ = fromEvent(button, 'mousedown');

// Einde: Muis omhoog
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `Gebeurtenissen geregistreerd tijdens vasthouden: ${events.length} items`;
  console.log('Geregistreerde data:', events);
});
```

## 🎯 Praktisch codevoorbeeld 2: Dataverzameling tijdens kantooruren

Dit is een voorbeeld van het verzamelen van sensordata van het begin van kantooruren tot het einde van kantooruren.

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, mergeMap, scan, map } from 'rxjs';

// Sensordata (altijd verzamelend)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10, // 20-30 graden
    humidity: 40 + Math.random() * 20     // 40-60%
  }))
);

// Kantoor open: na 2 seconden, dan elke 10 seconden
const businessOpen$ = timer(2000, 10000);

// Kantoor sluit: 5 seconden na start
const businessClose = () => timer(5000);

let sessionNumber = 0;

sensorData$.pipe(
  windowToggle(businessOpen$, businessClose),
  mergeMap(window$ => {
    const current = ++sessionNumber;
    console.log(`Kantoorsessie ${current} gestart`);

    // Bereken statistieken voor elk venster
    return window$.pipe(
      scan((stats, data) => ({
        count: stats.count + 1,
        totalTemp: stats.totalTemp + data.temperature,
        totalHumidity: stats.totalHumidity + data.humidity
      }), { count: 0, totalTemp: 0, totalHumidity: 0 }),
      map(stats => ({
        session: current,
        count: stats.count,
        avgTemp: stats.totalTemp / stats.count,
        avgHumidity: stats.totalHumidity / stats.count
      }))
    );
  })
).subscribe(stats => {
  console.log(`Sessie ${stats.session}: ${stats.count} samples`);
  console.log(`  Gemiddelde temperatuur: ${stats.avgTemp.toFixed(1)}°C`);
  console.log(`  Gemiddelde vochtigheid: ${stats.avgHumidity.toFixed(1)}%`);
});
```

## 🎯 Praktisch voorbeeld: Downloadperiodebeheer

Dit is een voorbeeld van het beheren van datadownloadperiodes met start- en stopknoppen.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { windowToggle, mergeMap, toArray, map } from 'rxjs';

// Maak UI-elementen
const startButton = document.createElement('button');
startButton.textContent = 'Start';
document.body.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
document.body.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Wachten...';
document.body.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
document.body.appendChild(result);

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

// Vensterbeheer
downloadData$.pipe(
  windowToggle(start$, () => stop$),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download voltooid</strong><br>
    Aantal: ${downloads.length} items<br>
    Totale grootte: ${(totalSize / 1024).toFixed(2)} MB<br>
    Gemiddelde grootte: ${avgSize.toFixed(0)} KB
  `;
});
```

## 🎯 Overlappende vensterperiodes

Een kenmerk van `windowToggle` is dat het meerdere vensterperiodes tegelijk kan beheren.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Start: elke 1 seconde
const opening$ = interval(1000);

// Einde: 1,5 seconden na start
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Venster:', values);
});

// Uitvoer:
// Venster: [4, 5, 6, 7]       (Start op 1 sec → Eindigt op 2,5 sec)
// Venster: [9, 10, 11, 12]    (Start op 2 sec → Eindigt op 3,5 sec)
// Venster: [14, 15, 16, 17]   (Start op 3 sec → Eindigt op 4,5 sec)
```

**Tijdlijn**:
```
Bron:      0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Start:     ----1sec----2sec----3sec----4sec
Periode 1: [------1.5sec-----]
            └→ Venster 1: [4,5,6,7]
Periode 2:        [------1.5sec-----]
                   └→ Venster 2: [9,10,11,12]
Periode 3:               [------1.5sec-----]
                          └→ Venster 3: [14,15,16,17]
```

## ⚠️ Opmerkingen

### 1. Venster abonnementbeheer

Elk venster is een onafhankelijke Observable, dus u moet er expliciet op abonneren of het plat maken met `mergeAll()` of vergelijkbaar.

```ts
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  // Waarden stromen niet tenzij u abonneert op het venster zelf
  window$.subscribe(value => {
    console.log('Waarde:', value);
  });
});
```

### 2. Pas op voor geheugenlekken

Als starttriggers te frequent zijn, zullen veel vensters tegelijk bestaan, wat geheugen verbruikt.

```ts
// ❌ Slecht voorbeeld: Start elke 100ms, eindig na 5 seconden
const opening$ = interval(100); // Te frequent
const closing = () => interval(5000);

source$.pipe(
  windowToggle(opening$, closing)
).subscribe();
// Tot 50 vensters kunnen tegelijk bestaan → Geheugenrisico

// ✅ Goed voorbeeld: Stel passend interval in
const opening$ = interval(2000); // Elke 2 seconden
const closing = () => interval(1000); // Voor 1 seconde
```

### 3. Overlappende vensterperiodes

Overlappende vensterperiodes resulteren in dezelfde waarde in meerdere vensters. Controleer of dit het beoogde gedrag is.

```ts
// Met overlap
opening$ = interval(1000);    // Start elke 1 seconde
closing = () => interval(1500); // Voor 1,5 seconden

// Zonder overlap
opening$ = interval(2000);    // Start elke 2 seconden
closing = () => interval(1000); // Voor 1 seconde
```

## 🆚 Vergelijking van window operators

| Operator | Controle | Vensterperiode | Gebruiksscenario |
|:---|:---|:---|:---|
| `window` | Andere Observable zendt uit | Continu | Event-gedreven partitionering |
| `windowTime` | Vast tijdsinterval | Continu | Tijd-gebaseerde partitionering |
| `windowCount` | Vast aantal | Continu | Aantal-gebaseerde partitionering |
| `windowToggle` | **Aparte start/eind controle** | **Kan overlappen** | **Complexe start/eind condities** |
| `windowWhen` | Alleen eind controle | Continu | Eenvoudige periodieke controle |

## 🔄 Verschil met windowWhen

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, windowWhen, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowToggle: Aparte controle van start en einde
source$.pipe(
  windowToggle(
    interval(1000),          // Starttrigger
    () => timer(500)         // Eindtrigger (500ms na start)
  ),
  mergeAll()
).subscribe();

// windowWhen: Controleer alleen eindtiming (volgende start onmiddellijk na einde)
source$.pipe(
  windowWhen(() => timer(1000)), // Venster elke 1 seconde
  mergeAll()
).subscribe();
```

| Operator | Controle | Vensterperiode | Gebruiksscenario |
|:---|:---|:---|:---|
| `windowToggle(open$, close)` | Aparte start/eind controle | Kan overlappen | Complexe start/eind condities |
| `windowWhen(closing)` | Alleen eind controle | Continu | Eenvoudig periodiek venster |

## 📚 Gerelateerde operators

- [bufferToggle](/nl/guide/operators/transformation/bufferToggle) - Verzamel waarden als array (array versie van windowToggle)
- [window](/nl/guide/operators/transformation/window) - Splits venster op timing van verschillende Observable
- [windowTime](/nl/guide/operators/transformation/windowTime) - Tijd-gebaseerde venstersplitsing
- [windowCount](/nl/guide/operators/transformation/windowCount) - Aantal-gebaseerde venstersplitsing
- [windowWhen](/nl/guide/operators/transformation/windowWhen) - Venstersplitsing met dynamische sluitcondities

## Samenvatting

De `windowToggle` operator is een geavanceerd hulpmiddel waarmee u de start en het einde onafhankelijk kunt controleren en elke periode als een onafhankelijke Observable kunt behandelen.

- ✅ Start en einde kunnen afzonderlijk worden gecontroleerd
- ✅ Meerdere vensters kunnen tegelijk worden beheerd
- ✅ Verschillende verwerking kan worden toegepast op elk venster
- ⚠️ Abonnementbeheer vereist
- ⚠️ Frequente starttriggers verbruiken geheugen
- ⚠️ Wees bewust van overlappende vensterperiodes
