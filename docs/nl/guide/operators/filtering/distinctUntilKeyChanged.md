---
description: De distinctUntilKeyChanged operator richt zich op een specifieke eigenschap binnen een objectstream en geeft alleen uit wanneer die waarde verschilt van de vorige. Het slaat efficiënt opeenvolgende dubbele data over en is nuttig voor het detecteren van statuswijzigingen en het optimaliseren van lijstupdates.
titleTemplate: ':title'
---

# distinctUntilKeyChanged - Detecteer alleen wijzigingen in specifieke eigenschap

De `distinctUntilKeyChanged` operator richt zich op een specifieke sleutel (eigenschap) van een object en geeft alleen uit wanneer die waarde verschilt van de vorige.
Het is nuttig voor het efficiënt overslaan van opeenvolgende duplicaten.


## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs';

const users = [
  { id: 1, name: 'Jansen' },
  { id: 2, name: 'Jansen' }, // Zelfde naam, overslaan
  { id: 3, name: 'De Vries' },
  { id: 4, name: 'Bakker' },
  { id: 5, name: 'Bakker' }, // Zelfde naam, overslaan
  { id: 6, name: 'Jansen' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// Output:
// { id: 1, name: 'Jansen' }
// { id: 3, name: 'De Vries' }
// { id: 4, name: 'Bakker' }
// { id: 6, name: 'Jansen' }
```

- Geeft alleen uit wanneer de waarde van de gespecificeerde eigenschap `name` verandert.
- Andere eigenschappen (bijv. `id`) worden niet vergeleken.

[🌐 RxJS Officiële Documentatie - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)


## 💡 Typische gebruikspatronen

- Lijstweergave alleen bijwerken wanneer een specifieke eigenschap verandert
- Detecteer alleen wijzigingen in specifieke attributen in gebeurtenisstreams
- Beheer dubbele verwijdering op sleutelbasis


## 🧠 Praktisch codevoorbeeld (met UI)

Voer een naam in in het tekstvak en druk op Enter om deze te registreren.
**Als dezelfde naam opeenvolgend wordt ingevoerd, wordt deze genegeerd**, en het wordt alleen aan de lijst toegevoegd wanneer een andere naam wordt ingevoerd.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'distinctUntilKeyChanged Praktisch Voorbeeld';
output.appendChild(title);

// Invoerformulier
const input = document.createElement('input');
input.placeholder = 'Voer naam in en druk Enter';
document.body.appendChild(input);

// Invoergebeurtenisstream
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `Gebruikersinvoer: ID=${user.id}, Naam=${user.name}`;
  output.appendChild(item);
});
```

- Als dezelfde naam opeenvolgend wordt ingevoerd, wordt deze overgeslagen.
- Het wordt alleen weergegeven wanneer een nieuwe naam wordt ingevoerd.
