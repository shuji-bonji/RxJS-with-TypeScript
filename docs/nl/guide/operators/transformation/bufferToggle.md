---
description: "De bufferToggle operator is een geavanceerde buffer operator waarmee begin- en eindtriggers kunnen worden geregeld door afzonderlijke Observable en waarmee meerdere bufferperioden onafhankelijk kunnen worden beheerd."
---

# bufferToggle - controlebuffer starten/eindigen

De `bufferToggle` operator controleert de **start trigger** en **einde trigger** met aparte Observable en geeft de waarden samen uit in een array. Dit is een geavanceerde bufferoperator die meerdere bufferperioden tegelijk kan beheren.

## 🔰 Basis syntaxis en gebruik

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // 0.5Waarde elke seconde uitgeven

// Start trigger: 2Elke seconde
const opening$ = interval(2000);

// Eind trigger: Vanaf begin1Seconden na
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Uitgang:
// [3, 4, 5]     (2Begin bij seconde,3(begin bij seconde, einde bij seconde)
// [7, 8, 9]     (4Begin bij seconde,5(begin bij seconde, einde bij seconde)
// [11, 12, 13]  (6Begin bij seconde,7(begin bij seconde, einde bij seconde)
```

**Bewerkingsstroom**:.
1. `opening$` geeft een waarde → buffering start
2. de door `closing()` geretourneerde Observable geeft een waarde af → einde buffering, output array
3. meerdere bufferperioden kunnen elkaar overlappen

[🌐 Officiële RxJS documentatie - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)

## 🆚 Contrast met andere buffer-gebaseerde operatoren

`bufferToggle` is uniek in vergelijking met andere buffer-gebaseerde operatoren omdat het **onafhankelijke controle van start en einde** toestaat.

### Vergelijking van verschillende operatoren

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // 0.5Waarde elke seconde uitgeven

// Start trigger: 2Elke seconde
const opening$ = interval(2000);

// Eind trigger: Vanaf begin1Seconden na
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Uitgang:
// [3, 4, 5]     (2Begin bij seconde,3(begin bij seconde, einde bij seconde)
// [7, 8, 9]     (4Begin bij seconde,5(begin bij seconde, einde bij seconde)
// [11, 12, 13]  (6Begin bij seconde,7(begin bij seconde, einde bij seconde)
```

### Vergelijking met codevoorbeelden


```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9elke seconde.300msUitgegeven elke

// bufferToggle: Onafhankelijke controle van begin en einde
const opening$ = interval(1000); // 1Start elke seconde
const closing = () => interval(500); // Vanaf begin500msEinde na

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Uitgang: [3, 4], [6, 7], [9]
//
// Tijdlijn:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [Start        Einde]  [Start        Einde]  [Start  Einde]
//                  └→ [3,4]           └→ [6,7]           └→ [9]
```

**Gebruik met andere operatoren**:.
- **`buffer`** → Voer een buffer uit telkens als de trigger Observable een waarde afgeeft.
- **`bufferTime`** → automatisch uitvoer van een buffer elke bepaalde tijd.
- **`bufferCount`** → voer een buffer uit wanneer het opgegeven aantal buffers is opgebouwd.
- **`bufferToggle`** → aparte begin- en eindcontrole, overlappende perioden mogelijk.

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // 0.5Waarde elke seconde uitgeven

// Start trigger: 2Elke seconde
const opening$ = interval(2000);

// Eind trigger: Vanaf begin1Seconden na
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Uitgang:
// [3, 4, 5]     (2Begin bij seconde,3(begin bij seconde, einde bij seconde)
// [7, 8, 9]     (4Begin bij seconde,5(begin bij seconde, einde bij seconde)
// [11, 12, 13]  (6Begin bij seconde,7(begin bij seconde, einde bij seconde)
```

> Voor meer informatie over elke operator, zie [buffer](. /buffer), [bufferTime](. /bufferTime), [bufferCount](. /bufferCount).

## Typische gebruikspatronen.

1. **Verzameling van gegevens tijdens kantooruren**.


```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Sensorgegevens (altijd verworven)
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // Begin van de bewerking: 9:00(simulatie): 2(na 1,5 seconden)
   const businessOpen$ = timer(2000, 10000); // 2Seconden later, dan10Elke seconde

   // Einde bedrijfsuren: Vanaf begin5Seconden na
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`Gegevens tijdens bedrijfsuren: ${data.length}Geval`);
     console.log(`Gemiddelde waarde: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **Gebeurtenisregistratie tijdens indrukken knop**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Vasthouden';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // Gegevensstroom
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // Start: Muis omlaag
   const mouseDown$ = fromEvent(button, 'mousedown');

   // Einde: Muis omhoog (mousedownKomt voor vanmouseuptot)
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `Gebeurtenis opgenomen tijdens hold: ${events.length}Geval`;
     console.log('Opgenomen gebeurtenissen:', events);
   });
   ```

3. **Actieve gebruikersactie opgenomen**
   ```ts
   import { fromEvent, merge, timer } from 'rxjs';
    mport { bufferToggle, map } from 'rxjs';

   // Actie gebruiker
   const clicks$ = fromEvent(document, 'click').pipe(
     map(() => ({ type: 'click' as const, timestamp: Date.now() }))
   );

   const scrolls$ = fromEvent(window, 'scroll').pipe(
     map(() => ({ type: 'scroll' as const, timestamp: Date.now() }))
   );

   const keypresses$ = fromEvent(document, 'keypress').pipe(
     map(() => ({ type: 'keypress' as const, timestamp: Date.now() }))
   );

   const actions$ = merge(clicks$, scrolls$, keypresses$);

   // Begin van actieve status: Eerste actie
   const activeStart$ = actions$;

   // Einde actieve status: 5Geen actie in seconden
   const activeEnd = () => timer(5000);

   actions$.pipe(
     bufferToggle(activeStart$, activeEnd)
   ).subscribe(bufferedActions => {
     console.log(`Actieve sessie: ${bufferedActions.length}Aantal acties`);
     const summary = bufferedActions.reduce((acc, action) => {
       acc[action.type] = (acc[action.type] || 0) + 1;
       return acc;
     }, {} as Record<string, number>);
     console.log('Indeling:', summary);
   });
   ```

## 🧠 Praktisch codevoorbeeld (downloadperioden beheren)

Dit is een voorbeeld van het beheren van gegevensdownloadperioden met start- en stopknoppen.

```

ts.
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// UI-elementen maken
const container = document.createElement('div');.
document.body.appendChild(container);

const titel = document.createElement('h3');
title.textContent = "Data Download Management";
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = "Start";
container.appendChild(startknop);

const stopButton = document.createElement('knop');
stopButton.textContent = 'Stop';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Wachten...'. ;
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(resultaat);

// Gegevensstroom (downloadgegevens worden elke seconde gegenereerd)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    grootte: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    tijdstempel: nieuwe Datum())
  }))
);

// Start en einde triggeren
const start$ = fromEvent(startknop, 'klik');
const stop$ = nieuw Subject();

fromEvent(stopButton, 'click').subscribe() => {
  stop$.next();
  status.textContent = 'Gestopt';
  startknop.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe() => {
  status.textContent = 'Downloaden...' ;
  startknop.disabled = true;
  stopButton.disabled = false;
});

// Bufferen
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((som, d) => som + d.grootte, 0);
  const avgSize = downloads.length > 0 ? totaleGrootte / downloads.lengte : 0;

  resultaat.innerHTML = `
    <strong>Downloads voltooid</strong><br>.
    Aantal downloads: ${downloads.length}<br>
    Totale grootte: ${(totalSize / 1024).toFixed(2)} MB<br>
    Gemiddelde grootte: ${avgSize.toFixed(0)} KB
  `;

  console.log('Gedownloade gegevens:', downloads);
});

```

## 🎯 Overlappende bufferperioden

`bufferToggle` Meerdere bufferperioden kunnen tegelijkertijd worden beheerd als functie van het

```

ts.
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// start: elke 1 seconde
const opening$ = interval(1000);

// einde: 1,5 seconde na start
const closing = () => interval(1500);

bron$.pipe(
  bufferToggle(openen$, sluiten)
).subscribe(console.log);
// Uitvoer:.
// [4, 5, 6] (begin van 1e seconde → einde van 2,5 seconden)
// [9, 10, 11, 12] (begin 2e seconde → einde 3,5 seconde) * gedeeltelijk gedupliceerd
// [14, 15, 16, 17] (begin 3e seconde → einde 4,5 seconde)

```

**Tijdlijn**:
```

Source: 0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Begint: ----1 seconden ----2 seconden ----3 seconden ----4 seconden
Periode 1: [------1.5 seconden-----]
            Uitvoer: [4,5,6].
Periode 2: [------1,5 seconden-----]
                   └→ Uitgang: [9,10,11,12].
Periode 3: [------1,5 seconden-----]
                          └→ Uitgang: [14,15,16,17]

```

## 📋 Typeveilig gebruik

TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generics in

```

ts.
import { Observable, Subject, interval } from 'rxjs';
import { bufferToggle, map } from 'rxjs';

interface MetricData {
  timestamp: datum;.
  cpu: getal;
  geheugen: getal; }
}

interface SessionControl {
  start$: Observable;
  stop$: Observable;
}

class MetricsCollector {
  private startSubject = new Subject();
  privé stopSubject = nieuw Subject();.

  start(): void {
    this.startSubject.next();
  }

  stop(): void {
    this.stopSubject.next(); }
  }

  collectMetrics(source$: Observable): Observable<MetricData[]> {
    return source$.pipe(
      bufferToggle(
        this.startSubject,.
        () => this.stopSubject
      )
    );
  }
}

// Gebruiksvoorbeeld
const metricsStream$ = interval(500).pipe(
  map() => ({
    timestamp: new Date(), cpu: Math.random() * 100, cpu.
    cpu: Math.random() * 100, memory.
    geheugen: Math.random() * 100
  } als MetricData))
);

const collector = nieuwe MetricsCollector();.

collector.collectMetrics(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avgCpu = metriek.reduce((som, m) => som + m.cpu, 0) / metriek.lengte;
    const avgMemory = metrics.reduce((som, m) => som + m.memory, 0) / metrics.length;
    console.log(`verzamelperiode: ${metrics.length} gevallen`);
    console.log(`Gemiddelde CPU: ${avgCpu.toFixed(1)}%`);
    console.log(`Gemiddeld geheugen: ${avgMemory.toFixed(1)}%`); console.log(`Gemiddeld geheugen: ${avgMemory.toFixed(1)}%`); }
  }
});

// Start na 3 seconden.
setTimeout() => {
  console.log('Verzameling gestart');
  collector.start();
}, 3000);

// Stop na 6 seconden
setTimeout() => {
  console.log('Verzameling gestopt');
  collector.stop();
}, 6000);

```

## 🔄 bufferWhen Verschillen tussen

`bufferToggle` en `bufferWhen` zijn vergelijkbaar, maar verschillen in de manier waarop ze worden gecontroleerd.

```

ts.
import { interval, timer } from 'rxjs';
import { bufferToggle, bufferWhen } from 'rxjs';

const source$ = interval(200);.

// bufferToggle: begin en einde afzonderlijk regelen
bron$.pipe(
  bufferToggle(
    interval(1000), // start trigger
    () => timer(500) // eind-trigger (500 ms na start)
  )
).subscribe(console.log);.

// bufferWhen: regelt alleen de timing van het einde (de volgende start onmiddellijk na het einde)
bron$.pipe(
  bufferWhen() => timer(1000)) // buffer elke seconde
).subscribe(console.log);.

```

| Operator | Controle | Bufferperiode | Gebruik |
|---|---|---|---|
| `bufferToggle(open$, close)` | Aparte controle van begin en einde | Duplicatie mogelijk | Complex begin/Einde voorwaarden |
| `bufferWhen(closing)` | Alleen einde gecontroleerd | Doorlopend | Eenvoudige cyclische buffer |

## ⚠️ Veelvoorkomende fouten

> [!WARNING]
> `bufferToggle` kan meerdere bufferperioden tegelijk beheren, maar als de starttrigger vaak afgaat, zullen er veel buffers tegelijk bestaan en geheugen verbruiken.

### Fout.: De starttriggers zijn te frequent.

```

ts.
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);.

// ❌ Slecht voorbeeld: start elke 100 ms, eindig na 5 seconden
const opening$ = interval(100); // te vaak
const closing = () => interval(5000);

bron$.pipe(
  bufferToggle(openen$, sluiten)
).subscribe(console.log);.
// Mogelijk 50 buffers tegelijk → geheugenrisico

```

### Corrigeer: Stel geschikte intervallen in

```

ts.
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);.

// ✅ Goed voorbeeld: begin met het juiste interval
const opening$ = interval(2000); // elke 2 seconden
const closing = () => interval(1000); // buffer voor 1 seconde

bron$.pipe(
  bufferToggle(openen$, sluiten)
).subscribe(console.log);.
// hooguit 1-2 buffers tegelijkertijd.
```

## Samenvatting

### Wanneer moet bufferToggle worden gebruikt?
- ✅ Als u begin en einde onafhankelijk wilt regelen
- ✅ Als u gegevens voor een beperkte periode wilt verzamelen, bijvoorbeeld tijdens het indrukken van een knop
- ✅ Als u meerdere bufferperioden tegelijk wilt beheren
- ✅ Als u gegevens wilt verzamelen onder complexe omstandigheden, bijvoorbeeld alleen tijdens kantooruren

### Wanneer u buffer/bufferTime/bufferCount moet gebruiken.
- ✅ Wanneer eenvoudige periodieke buffering voldoende is
- ✅ Wanneer een enkele trigger voldoende is voor controle

### Wanneer moet bufferWhen worden gebruikt?
- ✅ Wanneer alleen de eindtoestand dynamisch moet worden geregeld.
- ✅ Wanneer continue bufferperioden vereist zijn.

### Opmerkingen.
- ⚠️ Frequente starttriggers resulteren in veel buffers die tegelijkertijd bestaan, waardoor geheugen wordt verbruikt.
- ⚠️ Bufferperioden kunnen elkaar overlappen
- ⚠️ Kan moeilijk te debuggen zijn vanwege complexe besturingselementen

## Volgende stappen.

- **[buffer](. /buffer)** - basisbuffering leren.
- **[bufferTime](. /bufferTime)** - tijdgebaseerd bufferen leren.
- **[bufferCount](. /bufferCount)** - leer bufferen per stuk **[bufferCount](.
- **[bufferWhen](https://rxjs.dev/api/operators/bufferWhen)** - leer dynamische exitcontrole (officiële documentatie)
- **[conversion-operator-practical-use-cases](. /practical-use-cases)** - leer praktijkvoorbeelden
