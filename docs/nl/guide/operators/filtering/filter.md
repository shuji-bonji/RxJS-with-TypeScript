---
description: De filter operator selecteert waarden in een stream op basis van een opgegeven voorwaardefunctie en laat alleen waarden door die aan de voorwaarde voldoen. Deze filteringsoperator wordt gebruikt voor formulierinvoervalidatie, extractie van gegevens met specifieke voorwaarden, uitsluiting van null of undefined, enz., en maakt streams efficiënter door onnodige gegevens uit te sluiten. Het kan ook worden gebruikt als TypeScript type guard.
---

# filter - Waarden op Voorwaarde

De `filter` operator selecteert waarden in een stream op basis van een opgegeven voorwaardefunctie en laat alleen waarden door die aan de voorwaarde voldoen.

## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Output: 2, 4, 6, 8, 10
```

- Alleen waarden die aan de voorwaarde voldoen worden doorgelaten.
- Het werkt vergelijkbaar met `Array.prototype.filter()`, maar wordt sequentieel verwerkt op een Observable.

[🌐 RxJS Officiële Documentatie - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 Typische toepassingspatronen

- Validatie van formulierinvoerwaarden
- Alleen gegevens met een specifiek type of structuur toestaan
- Filteren van sensorgebeurtenissen of streamgegevens

## 🧠 Praktisch codevoorbeeld (met UI)

Toont een lijst in real-time alleen wanneer het ingevoerde getal even is.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'Praktijkvoorbeeld van filter:';
document.body.appendChild(title);

// Invoerveld aanmaken
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Voer een getal in';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Outputgebied aanmaken
const output = document.createElement('div');
document.body.appendChild(output);

// Invoergebeurtenisstream
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Even getal gedetecteerd: ${evenNumber}`;
    output.prepend(item);
  });

```

- Alleen wanneer het getal even is, wordt het weergegeven in de output.
- Oneven getallen en ongeldige invoer worden genegeerd.

> [!WARNING] Let op in productiecode
> Het bovenstaande voorbeeld laat het afmelden van `fromEvent` weg om de uitleg te vereenvoudigen. In echte code, beheer expliciet de levenscyclus met `takeUntil(destroy$)`, `take(N)`, of `Subscription.unsubscribe()`. Details: [Overwinnen van moeilijkheden: levenscyclusbeheer](/nl/guide/overcoming-difficulties/lifecycle-management.md)
