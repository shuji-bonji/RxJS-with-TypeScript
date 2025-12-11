---
description: De pairwise operator is een RxJS operator die twee opeenvolgende waarden als een array van paren uitvoert, en wordt gebruikt om de vorige waarde met de huidige waarde te vergelijken of om het verschil te berekenen.
titleTemplate: ':title | RxJS'
---

# pairwise - Verwerk twee opeenvolgende waarden als een paar

De `pairwise` operator **koppelt twee opeenvolgende waarden die van een stream worden uitgegeven als een array `[vorige waarde, huidige waarde]` en voert ze samen uit**.
Dit is nuttig voor het vergelijken van de vorige waarde met de huidige waarde of voor het berekenen van de hoeveelheid verandering.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Output:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- De eerste waarde (0) wordt niet alleen uitgevoerd, maar wordt uitgevoerd als `[0, 1]` wanneer de tweede waarde (1) aankomt.
- Altijd een paar van **vorige waarde en huidige waarde** wordt uitgevoerd.

[🌐 RxJS Officiële Documentatie - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 Typische gebruikspatronen

- Berekening van de hoeveelheid muis- of touchbeweging
- Berekening van de hoeveelheid verandering (verschil) in prijzen of waarden
- Statusveranderingsdetectie (vergelijking van vorige status en huidige status)
- Bepaling van scrollrichting

## 🧠 Praktisch codevoorbeeld (met UI)

Dit voorbeeld toont de richting en hoeveelheid muisbeweging.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Muisbewegingsgebeurtenis
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'Rechts' : deltaX < 0 ? 'Links' : 'Gestopt';

  output.innerHTML = `
    Vorige: (${prev.x}, ${prev.y})<br>
    Huidige: (${curr.x}, ${curr.y})<br>
    Beweging: Δx=${deltaX}, Δy=${deltaY}<br>
    Richting: ${direction}
  `;
});
```

- Wanneer de muis wordt bewogen, worden de vorige en huidige coördinaten en de hoeveelheid beweging weergegeven.
- Met `pairwise` kunnen de vorige en huidige coördinaten automatisch in paren worden verkregen.

## 🎯 Voorbeeld van het berekenen van de hoeveelheid verandering in een getal

Hier is een praktisch voorbeeld van het berekenen van de hoeveelheid verandering (verschil) in een numerieke waardestroom.

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (kwadraat getallen)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (verschil: +${result.diff})`);
});

// Output:
// 0 → 1 (verschil: +1)
// 1 → 4 (verschil: +3)
// 4 → 9 (verschil: +5)
// 9 → 16 (verschil: +7)
// 16 → 25 (verschil: +9)
```

## 🎯 Bepaling van scrollrichting

Hieronder volgt een voorbeeld van het bepalen van de scrollrichting (omhoog/omlaag).

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs';

// Maak vast weergave uitvoergebied
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// Dummy-inhoud voor scrollen
const content = document.createElement('div');
content.style.height = '200vh'; // Dubbele paginahoogte
content.innerHTML = '<h1>Scroll alstublieft naar beneden</h1>';
document.body.appendChild(content);

// Krijg scrollpositie
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // Throttle elke 100ms
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ Omlaag' : '↑ Omhoog';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} Scrollrichting: ${direction}<br>
    Vorige positie: ${prevY.toFixed(0)}px<br>
    Huidige positie: ${currY.toFixed(0)}px<br>
    Beweging: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- Als de pagina wordt gescrolld, worden de richting en positie-informatie weergegeven in een vast gebied rechtsboven.
- `pairwise` stelt u in staat om automatisch de vorige en huidige scrollpositie in paren te krijgen.

## 🎯 Type-veilig pairwise gebruiken

Dit is een voorbeeld van het benutten van TypeScript's type-inferentie.

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// Output:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 Vergelijking met bufferCount(2, 1)

`pairwise()` is equivalent aan `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Output: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Output: [1,2], [2,3], [3,4], [4,5]
```

**Gebruiksverschillen**:
- `pairwise()`: Behandelt expliciet paren van twee opeenvolgende waarden, en de intentie van de code is duidelijk
- `bufferCount(2, 1)`: Flexibeler (kan meer dan 3 venstergroottes aan)

## ⚠️ Opmerkingen

### De eerste waarde wordt niet uitgevoerd

Aangezien `pairwise` niets uitvoert tot de twee waarden uitgelijnd zijn, kan de eerste waarde niet alleen worden verkregen.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('Voltooid')
});

// Output:
// Voltooid
// (Geen waarden worden uitgevoerd)
```

**Tegenmaatregel**: Als u ook de eerste waarde wilt verwerken, voeg een initiële waarde toe met `startWith`.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Output:
// [0, 10]
// [10, 20]
// [20, 30]
```

### Geheugengebruik

Aangezien `pairwise` altijd slechts één vorige waarde bewaart, is het geheugenefficiënt.

## 📚 Gerelateerde operators

- [`scan`](/nl/guide/operators/transformation/scan) - Complexer accumulatieproces
- [`bufferCount`](/nl/guide/operators/transformation/bufferCount) - Vat waarden samen voor elk gespecificeerd aantal items
- [`distinctUntilChanged`](/nl/guide/operators/filtering/distinctUntilChanged) - Verwijder opeenvolgende dubbele waarden
- [`startWith`](/nl/guide/operators/utility/startWith) - Voeg initiële waarde toe

## Samenvatting

De `pairwise` operator voert twee opeenvolgende waarden uit als `[vorige waarde, huidige waarde]` paren. Dit is zeer nuttig voor **situaties waar een vergelijking van de vorige waarde en de huidige waarde nodig is**, zoals het volgen van muisbewegingen, het berekenen van prijsveranderingen en het detecteren van statusovergangen. Merk op dat de eerste waarde niet wordt uitgevoerd tot de tweede waarde aankomt, maar dit kan worden afgehandeld door een initiële waarde toe te voegen met `startWith`.
