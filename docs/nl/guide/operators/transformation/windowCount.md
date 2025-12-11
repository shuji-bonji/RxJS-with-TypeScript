---
description: windowCount is een RxJS conversieoperator die een Observable verdeelt op een gespecificeerd aantal items. Het is ideaal voor aantal-gebaseerde streamverwerking, aggregatie per vast aantal, en paginaverwerking. In tegenstelling tot bufferCount kan het onafhankelijke verwerking toepassen op elk venster. TypeScript type-inferentie maakt type-veilige venstersplitsing en streamoperaties mogelijk.
titleTemplate: ':title | RxJS'
---

# windowCount - Splits Observable op gespecificeerd aantal

De `windowCount` operator **verdeelt** uitgezonden waarden in nieuwe Observables voor elk gespecificeerd aantal.
Terwijl `bufferCount` een array retourneert, retourneert `windowCount` een **Observable&lt;T&gt;**, waardoor extra operators op elk venster kunnen worden toegepast.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// Geef waarden elke 100ms uit
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Plat elke venster
).subscribe(value => {
  console.log('Waarde in venster:', value);
});

// Uitvoer:
// Waarde in venster: 0
// Waarde in venster: 1
// Waarde in venster: 2
// Waarde in venster: 3
// Waarde in venster: 4
// (Nieuw venster start)
// Waarde in venster: 5
// ...
```

- Een nieuw venster (Observable) wordt gemaakt voor elke 5 waarden.
- Het is uniek doordat het verdeelt op een aantal basis.

[🌐 RxJS Officiële Documentatie - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Typische gebruikspatronen

- Aggregaatverwerking voor elk vast aantal
- Batch verzending van data (verschillende verwerking voor elk venster)
- Paginaverwerking
- Bereken statistieken per venster

## 🔍 Verschil met bufferCount

| Operator | Uitvoer | Gebruiksscenario |
|:---|:---|:---|
| `bufferCount` | **Array (T[])** | Gegroepeerde waarden samen verwerken |
| `windowCount` | **Observable&lt;T&gt;** | Verschillende streamverwerking voor elke groep |

```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - Uitvoer als array
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Uitvoer: Buffer (array): [0, 1, 2, 3, 4]
});

// windowCount - Uitvoer als Observable
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('Venster (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Waarde in venster:', value);
  });
});
```

## 🧠 Praktisch codevoorbeeld 1: Som per venster

Dit is een voorbeeld van het berekenen van de som van elke 5 waarden.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.innerHTML = '<h3>Som elke 5 waarden</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Venster ${current} gestart`);

    // Bereken som voor elk venster
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // Voeg vensternummer toe
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Venster ${result.windowNum} som: ${result.sum}`;
  output.appendChild(div);
});

// Uitvoer:
// Venster 1 som: 10  (0+1+2+3+4)
// Venster 2 som: 35  (5+6+7+8+9)
// Venster 3 som: 60  (10+11+12+13+14)
```

## 🎯 Praktisch codevoorbeeld 2: Startindex specificeren

U kunt een startindex specificeren met het tweede argument. Dit maakt overlappende vensters.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Geef waarden van 0 tot 9 uit
range(0, 10).pipe(
  windowCount(3, 2), // 3 items elk, start verschoven met 2
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Venster:', values);
});

// Uitvoer:
// Venster: [0, 1, 2]
// Venster: [2, 3, 4]    ← Gestart verschoven met 2 (vanaf 2)
// Venster: [4, 5, 6]    ← Gestart verschoven met 2 (vanaf 4)
// Venster: [6, 7, 8]
// Venster: [8, 9]       ← Laatste 2 items
```

### Startindex operatiepatronen

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Continu (standaard): [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Overlap: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // Met gat: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 Praktisch voorbeeld: Verschillende verwerking voor elk venster

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Even vensters: Krijg alleen eerste 2 items
      console.log(`Venster ${current}: Krijg eerste 2 items`);
      return window$.pipe(take(2));
    } else {
      // Oneven vensters: Krijg alles
      console.log(`Venster ${current}: Krijg alles`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Waarde: ${value} (Venster ${windowNumber})`);
});
```

## 🧠 Praktisch codevoorbeeld 3: Paginatie-achtige verwerking

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray, map } from 'rxjs';

// Data van 1-20
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// Pagineer met 5 items
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`Pagina ${page.page}:`, page.items);
});

// Uitvoer:
// Pagina 1: [1, 2, 3, 4, 5]
// Pagina 2: [6, 7, 8, 9, 10]
// Pagina 3: [11, 12, 13, 14, 15]
// Pagina 4: [16, 17, 18, 19, 20]
```

## ⚠️ Opmerkingen

### 1. Venster abonnementbeheer

Elk venster is een onafhankelijke Observable en moet expliciet worden geabonneerd.

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // Waarden stromen niet tenzij u abonneert op het venster zelf
  window$.subscribe(value => {
    console.log('Waarde:', value);
  });
});
```

Of gebruik `mergeAll()`, `concatAll()`, `switchAll()`, etc. om te plat te maken.

### 2. Laatste venster

Bij voltooiing van de bron Observable wordt het laatste venster uitgevoerd zelfs als het minder dan het gespecificeerde aantal items bevat.

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Venster:', values);
});

// Uitvoer:
// Venster: [1, 2, 3]
// Venster: [4, 5, 6]
// Venster: [7]  ← Slechts 1 item
```

### 3. Geheugengebruik door startindex

Als `startBufferEvery` kleiner is dan `bufferSize` (overlap), zullen meerdere vensters tegelijk actief zijn, wat het geheugengebruik verhoogt.

```ts
// Overlap: Maximaal 2 vensters gelijktijdig actief
windowCount(5, 3)

// Tegenmaatregel: Beperk met take() indien nodig
source$.pipe(
  take(100), // Maximaal 100 items
  windowCount(5, 3)
)
```

## 🆚 Vergelijking van window operators

| Operator | Timing van afbakening | Gebruiksscenario |
|:---|:---|:---|
| `window` | Andere Observable zendt uit | Event-gedreven partitionering |
| `windowTime` | Vast tijdsinterval | Tijd-gebaseerde partitionering |
| `windowCount` | **Vast aantal** | **Aantal-gebaseerde partitionering** |
| `windowToggle` | Start en eind Observables | Dynamische start/eind controle |
| `windowWhen` | Dynamische sluitconditie | Verschillende eindconditie per venster |

## 📚 Gerelateerde operators

- [`bufferCount`](/nl/guide/operators/transformation/bufferCount) - Verzamel waarden als array (array versie van windowCount)
- [`window`](/nl/guide/operators/transformation/window) - Splits venster op timing van verschillende Observable
- [`windowTime`](/nl/guide/operators/transformation/windowTime) - Tijd-gebaseerde venstersplitsing
- [`windowToggle`](/nl/guide/operators/transformation/windowToggle) - Venstercontrole met start en eind Observables
- [`windowWhen`](/nl/guide/operators/transformation/windowWhen) - Venstersplitsing met dynamische sluitcondities

## Samenvatting

De `windowCount` operator is een nuttig hulpmiddel voor het partitioneren van streams op een aantal basis en het behandelen van elke groep als een onafhankelijke Observable.

- ✅ Ideaal voor aggregatie en verwerking op vast aantal
- ✅ Verschillende verwerking kan worden toegepast op elk venster
- ✅ Kan worden overlapt door startindex
- ⚠️ Vereist abonnementbeheer
- ⚠️ Wees bewust van geheugengebruik bij overlapping
