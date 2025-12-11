---
description: bufferWhen is een RxJS conversieoperator die de eindconditie dynamisch controleert en waarden in een array publiceert. Het maakt continue buffering mogelijk waarbij de volgende buffer onmiddellijk start nadat de buffer eindigt, en kan worden gebruikt voor flexibele data-aggregatie gebaseerd op belasting, zoals adaptieve batchverwerking en logverzameling. TypeScript type-inferentie maakt type-veilige dynamische buffering mogelijk.
---

# bufferWhen - Buffer met dynamische eindcontrole

De `bufferWhen` operator publiceert een array van waarden met **dynamisch gecontroleerde eindcondities**. Het biedt een continu bufferpatroon waarbij één buffer eindigt en de volgende buffer onmiddellijk start.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(500); // Geef waarden elke 0,5 seconden uit

// Eindconditie: na 1 seconde
const closingSelector = () => interval(1000);

source$.pipe(
  bufferWhen(closingSelector),
  take(4)
).subscribe(console.log);
// Output:
// [0]           (Start op 0 sec → Eindigt op 1 sec, alleen waarde 0)
// [1, 2, 3]     (Start op 1 sec → Eindigt op 2 sec, waarden 1,2,3)
// [4, 5]        (Start op 2 sec → Eindigt op 3 sec, waarden 4,5)
// [6, 7]        (Start op 3 sec → Eindigt op 4 sec, waarden 6,7)
```

**Werkingsstroom**:
1. Eerste buffer start automatisch
2. Observable geretourneerd door `closingSelector()` zendt een waarde uit → Buffer eindigt, voert array uit
3. **Volgende buffer start onmiddellijk** (vaak tegelijk met source$ emissie)
4. Herhaal 2-3

> [!NOTE]
> De eerste buffer bevat alleen `[0]` omdat het de periode van 1 seconde is totdat `interval(1000)` zijn eerste waarde uitzendt. Vanaf de tweede buffer vallen bufferstart en `source$` emissie samen, dus bevatten ze meer waarden.

[🌐 RxJS Officiële Documentatie - `bufferWhen`](https://rxjs.dev/api/operators/bufferWhen)

## 🆚 Verschil met bufferToggle

`bufferWhen` en `bufferToggle` zijn vergelijkbaar, maar **hun controlemethoden en gedragspatronen zijn zeer verschillend**.

### bufferWhen gedrag

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Geef 0-11 elke 300ms uit

// bufferWhen: Controleer alleen einde (volgende start onmiddellijk na einde)
source$.pipe(
  bufferWhen(() => interval(1000))
).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8, 9], [10, 11]
//
// Tijdlijn:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms 3600ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//  [----------1sec----------][----------1sec----------][----------1sec----------][-----1sec-----]
//   Buffer1(0-2)              Buffer2(3-5)              Buffer3(6-9)             Buffer4(10-11)
//   Continu, geen overlap, volgende start onmiddellijk
```

### bufferToggle gedrag

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Geef 0-11 elke 300ms uit

// bufferToggle: Aparte controle van start en einde (kan overlappen)
const opening$ = interval(1000); // Start elke 1 seconde
const closing = () => interval(800); // Eindig 800ms na start

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4, 5], [6, 7, 8], [9, 10, 11]
//
// Tijdlijn:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//        ----Start1(1000ms)----[---Eind na 800ms(1800ms)---]
//                        3      4      5
//                        └→ [3,4,5]
//                    ----Start2(2000ms)----[---Eind na 800ms(2800ms)---]
//                                            6      7      8
//                                            └→ [6,7,8]
//                              ----Start3(3000ms)----[---Eind na 800ms(3800ms)---]
//                                                      9      10     11
//                                                      └→ [9,10,11]
//  Wacht op starttrigger, periodes zijn onafhankelijk (0-2 voor bufferstart niet opgenomen)
```

### Belangrijkste verschillen

| Operator | Startcontrole | Eindcontrole | Bufferperiode | Kenmerk |
|---|---|---|---|---|
| `bufferWhen(closing)` | Auto (onmiddellijk na einde) | Dynamisch | Continu | Geen gat tussen buffers |
| `bufferToggle(open$, close)` | Onafhankelijke Observable | Dynamisch | Onafhankelijk, kan overlappen | Gat tussen buffers |

**Gebruiksrichtlijnen**:
- **`bufferWhen`**: Buffer alle data continu zonder omissie (logging, data-aggregatie, etc.)
- **`bufferToggle`**: Verzamel data alleen tijdens specifieke periodes (tijdens kantooruren, knop indrukken, etc.)

> [!TIP]
> - **Continue buffering** (geen datalek) → `bufferWhen`
> - **Beperkte periode buffering** (expliciete start/eind controle) → `bufferToggle`

## 💡 Typische gebruikspatronen

1. **Dataverzameling op dynamische tijdsintervallen**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Sensordata
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       temperature: 20 + Math.random() * 10
     }))
   );

   // Eindconditie: Dynamisch veranderen op basis van vorige temperatuur
   let previousAvgTemp = 25;

   sensorData$.pipe(
     bufferWhen(() => {
       // Hogere temperatuur = korter bufferinterval
       const duration = previousAvgTemp > 27 ? 500 : 1000;
       return timer(duration);
     })
   ).subscribe(data => {
     const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
     previousAvgTemp = avgTemp;
     console.log(`Gem. temp: ${avgTemp.toFixed(1)}°C, Samples: ${data.length}`);
   });
   ```

2. **Adaptieve batchverwerking gebaseerd op belasting**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   interface Task {
     id: number;
     timestamp: number;
   }

   // Taakstroom
   let taskCounter = 0;
   const tasks$ = fromEvent(document, 'click').pipe(
     map(() => ({
       id: taskCounter++,
       timestamp: Date.now()
     } as Task))
   );

   // Pas volgende bufferperiode aan op basis van buffergrootte
   tasks$.pipe(
     bufferWhen(() => timer(2000))
   ).subscribe(bufferedTasks => {
     if (bufferedTasks.length > 0) {
       console.log(`Batchverwerking: ${bufferedTasks.length} taken`);
       console.log('Taak IDs:', bufferedTasks.map(t => t.id));

       // Bepaal dynamisch volgende bufferperiode
       // (In de praktijk, verplaats deze logica naar binnen de bufferWhen functie)
     }
   });
   ```

3. **Sampling op willekeurige intervallen**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Datastroom
   const data$ = interval(100).pipe(
     map(i => ({
       value: Math.sin(i / 10) * 100,
       timestamp: Date.now()
     }))
   );

   // Buffer op willekeurige intervallen (500ms ~ 2000ms)
   data$.pipe(
     bufferWhen(() => {
       const randomDelay = 500 + Math.random() * 1500;
       return timer(randomDelay);
     })
   ).subscribe(samples => {
     const avg = samples.reduce((sum, s) => sum + s.value, 0) / samples.length;
     console.log(`Aantal samples: ${samples.length}, Gemiddelde: ${avg.toFixed(2)}`);
   });
   ```

## 🧠 Praktisch codevoorbeeld (Belasting-gebaseerde logverzameling)

Dit is een voorbeeld van het dynamisch veranderen van logverzamelfrequentie op basis van systeembelasting.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { bufferWhen, map, share } from 'rxjs';

// Maak UI-elementen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Adaptief logverzamelsysteem';
container.appendChild(title);

const loadButton = document.createElement('button');
loadButton.textContent = 'Genereer belasting';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
status.textContent = 'Lage belasting: Verzamelen op 5-seconden intervallen';
container.appendChild(status);

const logDisplay = document.createElement('pre');
logDisplay.style.marginTop = '10px';
logDisplay.style.padding = '10px';
logDisplay.style.backgroundColor = '#f9f9f9';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Logstroom (altijd genererend)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    message: `Logbericht ${logCounter}`,
    timestamp: new Date()
  })),
  share()
);

// Belastingteller (verhoog bij knopklik)
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// Verlaag belasting elke 30 seconden
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getBufferInterval(loadLevel);
  const loadText = loadLevel === 0 ? 'Lage belasting' :
                   loadLevel <= 2 ? 'Gemiddelde belasting' : 'Hoge belasting';
  status.textContent = `${loadText} (Niveau ${loadLevel}): Verzamelen op ${interval / 1000}-seconden intervallen`;
  status.style.backgroundColor =
    loadLevel === 0 ? '#d4edda' :
    loadLevel <= 2 ? '#fff3cd' : '#f8d7da';
}

function getBufferInterval(load: number): number {
  // Hogere belasting = korter bufferinterval
  switch (load) {
    case 0: return 5000;  // 5 seconden
    case 1: return 3000;  // 3 seconden
    case 2: return 2000;  // 2 seconden
    case 3: return 1000;  // 1 seconde
    case 4: return 500;   // 0,5 seconden
    default: return 300;  // 0,3 seconden
  }
}

// Adaptieve buffering
logs$.pipe(
  bufferWhen(() => timer(getBufferInterval(loadLevel)))
).subscribe(bufferedLogs => {
  if (bufferedLogs.length > 0) {
    const errors = bufferedLogs.filter(log => log.level === 'ERROR').length;
    const timestamp = new Date().toLocaleTimeString();

    const summary = `[${timestamp}] Verzameld: ${bufferedLogs.length} items (Fouten: ${errors})\n`;
    logDisplay.textContent = summary + logDisplay.textContent;

    console.log('Verzamelde logs:', bufferedLogs);
  }
});
```

## 📋 Type-veilig gebruik

Hier is een voorbeeld van een type-veilige implementatie die generics in TypeScript gebruikt.

```ts
import { Observable, interval, timer } from 'rxjs';
import { bufferWhen, map } from 'rxjs';

interface MetricData {
  value: number;
  timestamp: Date;
  source: string;
}

interface BufferConfig {
  minDuration: number;
  maxDuration: number;
  adaptive: boolean;
}

class AdaptiveBuffer<T> {
  constructor(private config: BufferConfig) {}

  private getNextBufferDuration(previousCount: number): number {
    if (!this.config.adaptive) {
      return this.config.minDuration;
    }

    // Pas volgende bufferperiode aan op basis van datavolume
    const ratio = Math.min(previousCount / 10, 1);
    const duration =
      this.config.minDuration +
      (this.config.maxDuration - this.config.minDuration) * (1 - ratio);

    return Math.floor(duration);
  }

  apply(source$: Observable<T>): Observable<T[]> {
    let previousCount = 0;

    return source$.pipe(
      bufferWhen(() => {
        const duration = this.getNextBufferDuration(previousCount);
        return timer(duration);
      }),
      map(buffer => {
        previousCount = buffer.length;
        return buffer;
      })
    );
  }
}

// Gebruiksvoorbeeld
const metricsStream$ = interval(300).pipe(
  map(i => ({
    value: Math.random() * 100,
    timestamp: new Date(),
    source: `sensor-${i % 3}`
  } as MetricData))
);

const buffer = new AdaptiveBuffer<MetricData>({
  minDuration: 1000,  // Minimaal 1 seconde
  maxDuration: 5000,  // Maximaal 5 seconden
  adaptive: true      // Adaptief
});

buffer.apply(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avg = metrics.reduce((sum, m) => sum + m.value, 0) / metrics.length;
    console.log(`Buffergrootte: ${metrics.length}, Gemiddelde: ${avg.toFixed(2)}`);
  }
});
```

## 🎯 Vergelijking met andere buffer operators

```ts
import { interval, timer, Subject } from 'rxjs';
import { buffer, bufferTime, bufferCount, bufferWhen, bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9

// 1. buffer: Vaste trigger
const trigger$ = new Subject<void>();
source$.pipe(buffer(trigger$)).subscribe(console.log);
setInterval(() => trigger$.next(), 1000);
// Output: [0, 1, 2], [3, 4, 5], ... (op trigger timing)

// 2. bufferTime: Vast tijdsinterval
source$.pipe(bufferTime(1000)).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 3. bufferCount: Vast aantal
source$.pipe(bufferCount(3)).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 4. bufferWhen: Dynamische eindcontrole (continu)
source$.pipe(
  bufferWhen(() => timer(1000))
).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 5. bufferToggle: Onafhankelijke start/eind controle (kan overlappen)
const opening$ = interval(1000);
const closing = () => timer(800);
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4, 5], [6, 7, 8]
```

| Operator | Trigger | Dynamische controle | Overlap | Gebruiksscenario |
|---|---|---|---|---|
| `buffer` | Externe Observable | ❌ | ❌ | Event-gedreven |
| `bufferTime` | Vaste tijd | ❌ | ❌ | Periodieke aggregatie |
| `bufferCount` | Vast aantal | ❌ | ❌ | Kwantitatieve verwerking |
| `bufferWhen` | Dynamisch (alleen einde) | ✅ | ❌ | Adaptieve batchverwerking |
| `bufferToggle` | Dynamisch (start en einde) | ✅ | ✅ | Complexe periodebeheer |

## ⚠️ Veelgemaakte fouten

> [!WARNING]
> De `bufferWhen` eindconditiefunctie **moet elke keer een nieuwe Observable retourneren**. Als het dezelfde Observable-instantie retourneert, werkt het niet correct.

### Fout: Dezelfde Observable-instantie retourneren

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ❌ Slecht voorbeeld: Dezelfde Observable-instantie hergebruiken
const closingObservable = timer(1000);

source$.pipe(
  bufferWhen(() => closingObservable) // Werkt niet vanaf 2e keer!
).subscribe(console.log);
// Alleen de eerste buffer wordt uitgevoerd, daarna niets
```

### Correct: Retourneer elke keer een nieuwe Observable

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ✅ Goed voorbeeld: Genereer elke keer nieuwe Observable
source$.pipe(
  bufferWhen(() => timer(1000)) // Genereer elke keer nieuwe timer
).subscribe(console.log);
// Output: [0, 1], [2, 3], [4, 5], ...
```

> [!IMPORTANT]
> De `closingSelector` functie wordt **altijd aangeroepen** elke keer dat de vorige buffer eindigt, en er wordt verwacht dat het een nieuwe Observable retourneert.

## 🎓 Samenvatting

### Wanneer bufferWhen gebruiken
- ✅ Wanneer u de eindconditie dynamisch wilt controleren
- ✅ Wanneer continue bufferperiodes nodig zijn
- ✅ Wanneer u de volgende periode wilt aanpassen op basis van vorige bufferresultaten
- ✅ Wanneer u adaptieve batchverwerking wilt implementeren

### Wanneer bufferToggle gebruiken
- ✅ Wanneer u start en einde onafhankelijk wilt controleren
- ✅ Wanneer bufferperiodes kunnen overlappen
- ✅ Wanneer er duidelijke start/eind gebeurtenissen zijn, zoals knop indrukken

### Wanneer bufferTime gebruiken
- ✅ Wanneer buffering op vaste tijdsintervallen voldoende is
- ✅ Wanneer een eenvoudige implementatie vereist is

### Opmerkingen
- ⚠️ `closingSelector` moet elke keer een nieuwe Observable retourneren
- ⚠️ Te complexe eindcondities maken debuggen moeilijk
- ⚠️ Bij adaptieve controles is testen belangrijk om onverwacht gedrag te voorkomen

## 🚀 Volgende stappen

- [buffer](/nl/guide/operators/transformation/buffer) - Leer basis buffering
- [bufferTime](/nl/guide/operators/transformation/bufferTime) - Leer tijd-gebaseerde buffering
- [bufferCount](/nl/guide/operators/transformation/bufferCount) - Leer aantal-gebaseerde buffering
- [bufferToggle](/nl/guide/operators/transformation/bufferToggle) - Leer buffering met onafhankelijke start- en eindcontroles
- [Transformatieoperator praktische use cases](/nl/guide/operators/transformation/practical-use-cases) - Leer echte use cases
