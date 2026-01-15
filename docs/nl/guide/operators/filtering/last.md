---
description: "De last operator haalt alleen de laatste waarde bij streamvoltooiing of de laatste waarde die aan een voorwaarde voldoet: Essentieel voor eindstatusextractie"
titleTemplate: ':title'
---

# last - Haal Laatste Waarde

De `last` operator haalt de **laatste waarde** of **laatste waarde die aan een voorwaarde voldoet** uit de stream en voltooit de stream.


## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { last } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Haal alleen de laatste waarde
numbers$.pipe(
  last()
).subscribe(console.log);

// Haal alleen de laatste waarde die aan de voorwaarde voldoet
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// Output:
// 5
// 4
```

- `last()` geeft de **laatst uitgegeven waarde** uit bij streamvoltooiing.
- Als een voorwaarde wordt doorgegeven, wordt alleen de **laatste waarde** die aan de voorwaarde voldoet opgehaald.
- Als er geen waarde bestaat die aan de voorwaarde voldoet, wordt een fout gegenereerd.

[🌐 RxJS Officiële Documentatie - `last`](https://rxjs.dev/api/operators/last)


## 💡 Typische gebruikspatronen

- Haal het laatste element van gefilterde data
- Haal de laatste status bij streamvoltooiing op
- Haal de laatste significante operatie in de sessie of operatielog op


## 🧠 Praktisch codevoorbeeld (met UI)

Haal de laatste waarde op en toon deze die kleiner was dan 5 uit de meerdere ingevoerde waarden.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.innerHTML = '<h3>last Praktisch Voorbeeld:</h3>';
document.body.appendChild(output);

// Maak invoerveld
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Voer een getal in en druk Enter';
document.body.appendChild(input);

// Invoergebeurtenisstream
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // Voltooi wanneer alleen de eerste 5 waarden zijn genomen
    filter((n) => !isNaN(n) && n < 5), // Laat alleen waarden kleiner dan 5 door
    last() // Haal de laatste waarde kleiner dan 5
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `Laatste waarde kleiner dan 5: ${value}`;
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
1. Voer 5 keer een getal in en druk Enter
2. Pak alleen "kleiner dan 5" uit de ingevoerde getallen
3. Toon alleen het laatst ingevoerde getal dat kleiner is dan 5
4. De stream voltooit natuurlijk en eindigt
