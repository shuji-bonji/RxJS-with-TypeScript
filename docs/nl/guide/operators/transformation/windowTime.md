---
description: windowTime is een RxJS operator die een Observable kan verdelen op regelmatige tijdsintervallen en waarden uitgegeven in elk tijdsframe kan verwerken als een aparte Observable.
titleTemplate: ':title | RxJS'
---

# windowTime - Venster per Tijdseenheid

De `windowTime` operator groepeert de waarden van de bron Observable **op regelmatige intervallen** en geeft die groep uit als een **nieuwe Observable**.
Terwijl `bufferTime` een array retourneert, retourneert `windowTime` een **Observable&lt;T&gt;**, waardoor verdere operators op elk venster kunnen worden toegepast.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// Geef waarden elke 100ms uit
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // Maak venster elke 1 seconde
  take(3),          // Alleen eerste 3 vensters
  mergeAll()        // Plat elke venster
).subscribe(value => {
  console.log('Waarde:', value);
});

// Uitvoer:
// 1e seconde: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2e seconde: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3e seconde: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- Een nieuw venster (Observable) wordt gemaakt elke gespecificeerde tijd (1000ms).
- Elk venster kan worden verwerkt als een onafhankelijke Observable.

[🌐 RxJS Officiële Documentatie - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Typische gebruikspatronen

- **Tijd-gebaseerde batchverwerking**: Data wordt in batches verwerkt op regelmatige intervallen
- **Aggregeer realtime data**: Tel het aantal gebeurtenissen per seconde
- **Prestatiemonitoring**: Verzamel metrieken op regelmatige intervallen
- **Analyse van tijdreeksdata**: Statistische verwerking per tijdsframe

## 🔍 Verschil met bufferTime

| Operator | Uitvoer | Gebruiksscenario |
|:---|:---|:---|
| `bufferTime` | **Array (T[])** | Gegroepeerde waarden samen verwerken |
| `windowTime` | **Observable&lt;T&gt;** | Verschillende streamverwerking voor elk tijdsframe |

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - Uitvoer als array
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Uitvoer: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - Uitvoer als Observable
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('Venster (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Waarde:', value);
  });
});
```

## 🧠 Praktisch codevoorbeeld 1: Tel klikken per seconde

Dit is een voorbeeld van het tellen van het aantal klikken op een knop elke seconde.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Maak knop
const button = document.createElement('button');
button.textContent = 'Klik';
document.body.appendChild(button);

// Uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klikgebeurtenis
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // Maak venster elke 1 seconde
  map(window$ => {
    ++windowNumber;

    // Tel klikken in elk venster
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Venster ${windowNumber}: ${count} klikken`;
});
```

- Een nieuw venster wordt elke seconde gemaakt.
- Het aantal klikken in elk venster wordt realtime geteld.

## 🎯 Praktisch codevoorbeeld 2: Statistische verwerking per tijdsframe

Dit voorbeeld berekent de som en het gemiddelde van de waarden voor elk tijdsframe.

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.innerHTML = '<h3>Statistische verwerking per tijdsframe (elke 1 seconde)</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">Venster</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Aantal</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Som</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Gemiddelde</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // Willekeurige waarde
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // Elke 1 seconde
  take(5),          // Alleen 5 vensters
  mergeMap(window$ => {
    const current = ++windowNumber;

    // Converteer waarden in elk venster naar array en verwerk statistieken
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- Statistieken voor elk venster kunnen afzonderlijk worden berekend.
- Verschillende verwerking kan worden toegepast op elk venster.
- Statistieken worden visueel weergegeven in tabelformaat.

## 📊 Overlappende vensters (windowCreationInterval)

U kunt vensters overlappen door `windowCreationInterval` als tweede argument te specificeren.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.innerHTML = '<h3>Overlappende vensters</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Vensterlengte: 2 seconden
    1000   // Venster creatie-interval: 1 seconde
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `Venster ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `Waarden: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length} waarden, ${(result.window - 1)} sec ~ ${(result.window + 1)} sec)`;
  div.appendChild(info);

  output.appendChild(div);

  // Chrome workaround: Forceer rendering
  void output.offsetHeight;
});
```

**Hoe het werkt:**
- **Venster 1**: Waarden van 0 tot 2 seconden `[0, 1, 2, ..., 19]` (20 waarden)
- **Venster 2**: Waarden van 1 tot 3 seconden `[10, 11, 12, ..., 29]` (20 waarden) ← Waarden 10-19 overlappen met Venster 1
- **Venster 3**: Waarden van 2 tot 4 seconden `[20, 21, 22, ..., 39]` (20 waarden) ← Waarden 20-29 overlappen met Venster 2

- Het maken van een nieuw venster met een interval (1 seconde) korter dan de vensterlengte (2 seconden) resulteert in overlap.
- Nuttig voor schuifvenster implementaties.

## 🎯 Praktisch voorbeeld: Realtime gebeurtenismonitoring

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// Uitvoergebied
const output = document.createElement('div');
output.innerHTML = '<h3>Muisbeweging monitoring (elke 5 seconden)</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// Muisbeweging gebeurtenis
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // Elke 5 seconden
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] Muisbewegingen: ${result.count} keer`;
  list.insertBefore(item, list.firstChild);

  // Toon maximaal 10 items
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ Opmerkingen

### 1. Venster abonnementbeheer

Elk venster is een onafhankelijke Observable en moet expliciet worden geabonneerd.

```ts
source$.pipe(
  windowTime(1000)
).subscribe(window$ => {
  // Waarden stromen niet tenzij u abonneert op het venster zelf
  window$.subscribe(value => {
    console.log('Waarde:', value);
  });
});
```

Of gebruik `mergeAll()`, `concatAll()`, `switchAll()`, etc. om te plat te maken.

```ts
source$.pipe(
  windowTime(1000),
  mergeAll() // Voeg alle vensters samen
).subscribe(value => {
  console.log('Waarde:', value);
});
```

### 2. Geheugenbeheer

Bij langdurige uitvoering is het belangrijk om correct af te melden.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Afmelden bij vernietiging
).subscribe();

// Wanneer component wordt vernietigd, etc.
destroy$.next();
destroy$.complete();
```

### 3. Specificeer maximale waarde (maxWindowSize)

Het derde argument stelt u in staat om het maximale aantal waarden in elk venster te beperken.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray } from 'rxjs';

interval(100).pipe(
  windowTime(
    2000,      // Vensterlengte: 2 seconden
    undefined, // Venster creatie-interval: standaard (geen overlap)
    5          // Max waarde aantal: tot 5
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Venster:', values);
  // Bevat maximaal slechts 5 waarden
});
```

## 🆚 Vergelijking van window operators

| Operator | Timing van afbakening | Gebruiksscenario |
|:---|:---|:---|
| `window` | Andere Observable zendt uit | Event-gedreven partitionering |
| `windowTime` | **Vast tijdsinterval** | **Tijd-gebaseerde partitionering** |
| `windowCount` | Vast aantal | Aantal-gebaseerde partitionering |
| `windowToggle` | Start en eind Observables | Dynamische start/eind controle |
| `windowWhen` | Dynamische sluitconditie | Verschillende eindconditie per venster |

## 📚 Gerelateerde operators

- [bufferTime](/nl/guide/operators/transformation/bufferTime) - Verzamel waarden als array (array versie van windowTime)
- [window](/nl/guide/operators/transformation/window) - Splits venster op timing van verschillende Observable
- [windowCount](/nl/guide/operators/transformation/windowCount) - Aantal-gebaseerde venstersplitsing
- [windowToggle](/nl/guide/operators/transformation/windowToggle) - Venstercontrole met start en eind Observables
- [windowWhen](/nl/guide/operators/transformation/windowWhen) - Venstersplitsing met dynamische sluitcondities

## Samenvatting

De `windowTime` operator is een krachtig hulpmiddel voor het splitsen van streams op tijdbasis en het behandelen van elk tijdsframe als een onafhankelijke Observable.

- ✅ Maakt automatisch vensters op regelmatige intervallen
- ✅ Verschillende verwerking kan worden toegepast op elk venster
- ✅ Ondersteunt schuifvensters (overlap)
- ✅ Ideaal voor realtime data-aggregatie en -analyse
- ⚠️ Abonnementbeheer vereist
- ⚠️ Wees bewust van geheugenbeheer
