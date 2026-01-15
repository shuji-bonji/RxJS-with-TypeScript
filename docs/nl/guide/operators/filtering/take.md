---
description: De take operator haalt alleen het eerste gespecificeerde aantal waarden op uit de Observable-stream en voltooit de stream automatisch, waarbij volgende waarden worden genegeerd. Dit is nuttig wanneer u alleen de eerste paar stukken data wilt ophalen.
titleTemplate: ':title | RxJS'
---

# take - Eerste N Waarden

De `take` operator haalt alleen het **eerste gespecificeerde aantal** waarden op uit de stream en negeert volgende waarden.
Na voltooiing `voltooit` de stream automatisch.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2
```

- Abonneert alleen op de eerste 3 waarden.
- Na het ophalen van 3 waarden `voltooit` de Observable automatisch.

[🌐 RxJS Officiële Documentatie - `take`](https://rxjs.dev/api/operators/take)

## 💡 Typische gebruikspatronen

- Toon of log alleen de eerste paar items in UI
- Tijdelijk abonnement om alleen de eerste respons op te halen
- Beperkt ophalen van test- of demodata

## 🧠 Praktisch codevoorbeeld (met UI)

Haalt alleen de eerste 5 waarden op uit getallen die elke seconde worden uitgegeven en toont deze.

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.innerHTML = '<h3>take Praktisch Voorbeeld:</h3>';
document.body.appendChild(output);

// Geef waarden elke seconde uit
const source$ = interval(1000);

// Neem alleen de eerste 5 waarden
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `Waarde: ${value}`;
    output.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = 'Voltooid';
    complete.style.fontWeight = 'bold';
    output.appendChild(complete);
  },
});

```

- De eerste 5 waarden (`0`, `1`, `2`, `3`, `4`) worden op volgorde weergegeven,
- Daarna wordt het bericht "Voltooid" weergegeven.
