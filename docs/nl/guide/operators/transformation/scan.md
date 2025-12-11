---
description: De scan operator is een RxJS operator die tussenresultaten uitvoert terwijl elke waarde sequentieel wordt geaccumuleerd, en wordt gebruikt voor realtime aggregatie en statusbeheer.
---

# scan - Genereer waarden cumulatief

De `scan` operator past een cumulatieve functie toe op elke waarde in de stream en voert **sequentiële tussenresultaten** uit.
Vergelijkbaar met `Array.prototype.reduce` voor arrays, behalve dat het tussenresultaat sequentieel wordt uitgevoerd voordat alle waarden bereikt zijn.

## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Output: 1, 3, 6, 10, 15

```

- `acc` is de cumulatieve waarde, `curr` is de huidige waarde.
- Het begint met een initiële waarde (`0` in dit geval) en accumuleert sequentieel.

[🌐 RxJS Officiële Documentatie - `scan`](https://rxjs.dev/api/operators/scan)

## 💡 Typische gebruikspatronen

- Optellen en score-aggregatie
- Realtime formuliervalidatie statusbeheer
- Cumulatieve verwerking van gebufferde gebeurtenissen
- Dataconstructie voor realtime aggregatiegrafieken

## 🧠 Praktisch codevoorbeeld (met UI)

Toon het cumulatieve aantal klikken elke keer dat een knop wordt geklikt.

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// Maak knop
const button = document.createElement('button');
button.textContent = 'Klik';
document.body.appendChild(button);

// Maak uitvoergebied
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// Accumuleer klikgebeurtenissen
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `Kliktelling: ${count}`;
  });
```

- Elke keer dat een knop wordt geklikt, wordt de teller met 1 verhoogd.
- Door `scan` te gebruiken, kunt u **eenvoudige tellogica zonder statusbeheer** schrijven.
