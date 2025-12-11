---
description: De windowWhen operator is een RxJS operator die de eindconditie dynamisch controleert en Observable splitst. Het maakt continue streamverwerking mogelijk waarbij het volgende venster onmiddellijk start nadat het venster eindigt.
---

# windowWhen - Venster met dynamische eindcontrole

De `windowWhen` operator verdeelt Observable met **dynamische controle van eindcondities**. Het biedt een continu streamverwerkingspatroon waarbij het volgende venster onmiddellijk start nadat het venster eindigt.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // Geef waarden elke 0,5 seconden uit

// Eindconditie: na 1 seconde
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Waarde in venster:', value);
});

// Venster 1: 0       (Start op 0 sec → Eindigt op 1 sec)
// Venster 2: 1, 2    (Start op 1 sec → Eindigt op 2 sec)
// Venster 3: 3, 4    (Start op 2 sec → Eindigt op 3 sec)
// Venster 4: 5, 6    (Start op 3 sec → Eindigt op 4 sec)
```

**Werkingsstroom**:
1. Eerste venster start automatisch
2. Observable geretourneerd door `closingSelector()` zendt een waarde uit → Venster eindigt
3. **Volgende venster start onmiddellijk**
4. Herhaal 2-3

[🌐 RxJS Officiële Documentatie - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 Typische gebruikspatronen

- Dataverzameling op dynamische tijdsintervallen
- Adaptieve streamverwerking gebaseerd op belasting
- Venstercontrole gebaseerd op vorige resultaten
- Continue datagroepering

## 🔍 Verschil met bufferWhen

| Operator | Uitvoer | Gebruiksscenario |
|:---|:---|:---|
| `bufferWhen` | **Array (T[])** | Gegroepeerde waarden samen verwerken |
| `windowWhen` | **Observable&lt;T&gt;** | Verschillende streamverwerking voor elke groep |

```ts
import { interval } from 'rxjs';
import { bufferWhen, windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500);
const closing = () => interval(1000);

// bufferWhen - Uitvoer als array
source$.pipe(
  bufferWhen(closing),
  take(3)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Uitvoer: Buffer (array): [0]
  // Uitvoer: Buffer (array): [1, 2]
  // Uitvoer: Buffer (array): [3, 4]
});

// windowWhen - Uitvoer als Observable
source$.pipe(
  windowWhen(closing),
  take(3),
  mergeAll()
).subscribe(value => {
  console.log('Waarde in venster:', value);
  // Uitvoer: Waarde in venster: 0
  // Uitvoer: Waarde in venster: 1
  // Uitvoer: Waarde in venster: 2
  // ...
});
```

## 🧠 Praktisch codevoorbeeld 1: Dataverzameling op dynamische tijdsintervallen

Dit is een voorbeeld van het aanpassen van de volgende vensterperiode op basis van de resultaten van het vorige venster.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, scan, map } from 'rxjs';

// Sensordata (altijd genererend)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10 // 20-30 graden
  }))
);

let windowNumber = 0;
let previousAvgTemp = 25;

sensorData$.pipe(
  windowWhen(() => {
    const current = ++windowNumber;
    // Korter interval bij hogere temperatuur
    const duration = previousAvgTemp > 27 ? 500 : 1000;
    console.log(`Venster ${current} gestart (duur: ${duration}ms)`);
    return timer(duration);
  }),
  mergeMap(window$ => {
    const currentWindow = windowNumber;  // Bewaar huidig vensternummer
    return window$.pipe(
      toArray(),
      map(data => {
        const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
        previousAvgTemp = avgTemp;
        return {
          window: currentWindow,  // Gebruik bewaard vensternummer
          count: data.length,
          avgTemp
        };
      })
    );
  })
).subscribe(stats => {
  console.log(`Venster ${stats.window}: Gem. temp ${stats.avgTemp.toFixed(1)}°C, ${stats.count} samples`);
});
```

## 🎯 Praktisch codevoorbeeld 2: Adaptieve streamverwerking gebaseerd op belasting

Dit is een voorbeeld van het dynamisch veranderen van vensterlengte op basis van systeembelasting.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { windowWhen, mergeMap, scan, map } from 'rxjs';

// Maak uitvoergebied
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = 'Genereer belasting';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Lage belasting: Verzamelen op 5-seconden intervallen';
container.appendChild(status);

const logDisplay = document.createElement('div');
logDisplay.style.marginTop = '10px';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Logstroom (altijd genererend)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    timestamp: new Date()
  }))
);

// Belastingniveau
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
  const interval = getWindowDuration(loadLevel);
  const loadText = loadLevel === 0 ? 'Lage belasting' :
                   loadLevel <= 2 ? 'Gemiddelde belasting' : 'Hoge belasting';
  status.textContent = `${loadText} (Niveau ${loadLevel}): Verzamelen op ${interval / 1000}-seconden intervallen`;
}

function getWindowDuration(load: number): number {
  // Hogere belasting = korter interval
  switch (load) {
    case 0: return 5000;
    case 1: return 3000;
    case 2: return 2000;
    case 3: return 1000;
    case 4: return 500;
    default: return 300;
  }
}

let windowNum = 0;

// Adaptieve vensterverwerking
logs$.pipe(
  windowWhen(() => {
    windowNum++;
    return timer(getWindowDuration(loadLevel));
  }),
  mergeMap(window$ =>
    window$.pipe(
      scan((stats, log) => ({
        count: stats.count + 1,
        errors: stats.errors + (log.level === 'ERROR' ? 1 : 0),
        window: windowNum
      }), { count: 0, errors: 0, window: windowNum })
    )
  )
).subscribe(stats => {
  const timestamp = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.textContent = `[${timestamp}] Venster ${stats.window}: ${stats.count} items (Fouten: ${stats.errors})`;
  logDisplay.insertBefore(div, logDisplay.firstChild);
});
```

## 🆚 Verschil met windowToggle

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowWhen: Controleer alleen einde (volgende start onmiddellijk na einde)
source$.pipe(
  windowWhen(() => timer(1000)),
  mergeAll()
).subscribe();

// windowToggle: Aparte controle van start en einde
source$.pipe(
  windowToggle(
    interval(1000),          // Starttrigger
    () => timer(500)         // Eindtrigger (500ms na start)
  ),
  mergeAll()
).subscribe();
```

| Operator | Controle | Vensterperiode | Gebruiksscenario |
|:---|:---|:---|:---|
| `windowWhen(closing)` | Alleen eind controle | Continu | Eenvoudig periodiek venster |
| `windowToggle(open$, close)` | Aparte start/eind controle | Kan overlappen | Complexe start/eind condities |

**Gebruiksrichtlijnen**:
- **`windowWhen`**: Verwerk alle data continu zonder omissie (logging, data-aggregatie, etc.)
- **`windowToggle`**: Verwerk data alleen voor een specifieke periode (tijdens kantooruren, knop indrukken, etc.)

## 🎯 Praktisch voorbeeld: Adaptieve venstergrootte controle

Hier is een voorbeeld van het automatisch aanpassen van de volgende vensterperiode op basis van de resultaten van het vorige venster.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// Pas volgende vensterperiode aan op basis van datavolume
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // Hoog datavolume → Kort interval
  } else if (count > 10) {
    return 1000; // Gemiddeld → Gemiddeld interval
  } else {
    return 2000; // Laag datavolume → Lang interval
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`Venstergrootte: ${stats.count} items, Volgende duur: ${stats.nextDuration}ms`);
});
```

## ⚠️ Opmerkingen

### 1. Venster abonnementbeheer

Elk venster is een onafhankelijke Observable, dus u moet er expliciet op abonneren of het plat maken met `mergeAll()` of vergelijkbaar.

```ts
source$.pipe(
  windowWhen(closing)
).subscribe(window$ => {
  // Waarden stromen niet tenzij u abonneert op het venster zelf
  window$.subscribe(value => {
    console.log('Waarde:', value);
  });
});
```

### 2. Elke keer een nieuwe Observable retourneren

De `closingSelector` functie **moet elke keer een nieuwe Observable retourneren**. Als het dezelfde instantie retourneert, werkt het niet correct.

```ts
// ❌ Slecht voorbeeld: Dezelfde Observable-instantie hergebruiken
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // Werkt niet vanaf 2e keer!
).subscribe();

// ✅ Goed voorbeeld: Genereer elke keer nieuwe Observable
source$.pipe(
  windowWhen(() => timer(1000)) // Genereer elke keer nieuwe timer
).subscribe();
```

### 3. Pas op voor te complexe eindcondities

Te complexe eindcondities kunnen debuggen moeilijk maken.

```ts
// Te complex voorbeeld
let counter = 0;
source$.pipe(
  windowWhen(() => {
    counter++;
    const duration = counter % 3 === 0 ? 500 :
                     counter % 2 === 0 ? 1000 : 1500;
    return timer(duration);
  })
).subscribe();
// Moeilijk te debuggen
```

## 🆚 Vergelijking van window operators

| Operator | Controle | Vensterperiode | Gebruiksscenario |
|:---|:---|:---|:---|
| `window` | Andere Observable zendt uit | Continu | Event-gedreven partitionering |
| `windowTime` | Vast tijdsinterval | Continu | Tijd-gebaseerde partitionering |
| `windowCount` | Vast aantal | Continu | Aantal-gebaseerde partitionering |
| `windowToggle` | Aparte start/eind controle | Kan overlappen | Complexe start/eind condities |
| `windowWhen` | **Alleen dynamische eind controle** | **Continu** | **Adaptieve vensterverwerking** |

## 📚 Gerelateerde operators

- [bufferWhen](/nl/guide/operators/transformation/bufferWhen) - Verzamel waarden als array (array versie van windowWhen)
- [window](/nl/guide/operators/transformation/window) - Splits venster op timing van verschillende Observable
- [windowTime](/nl/guide/operators/transformation/windowTime) - Tijd-gebaseerde venstersplitsing
- [windowCount](/nl/guide/operators/transformation/windowCount) - Aantal-gebaseerde venstersplitsing
- [windowToggle](/nl/guide/operators/transformation/windowToggle) - Venstercontrole met start en eind Observables

## Samenvatting

De `windowWhen` operator is een nuttig hulpmiddel voor het dynamisch controleren van eindcondities en continue vensterverwerking.

- ✅ Eindcondities kunnen dynamisch worden gecontroleerd
- ✅ Continue vensterverwerking (geen datalek)
- ✅ Kan volgend venster aanpassen op basis van vorige resultaten
- ⚠️ Abonnementbeheer vereist
- ⚠️ Moet elke keer een nieuwe Observable retourneren
- ⚠️ Pas op dat eindcondities niet te complex worden
