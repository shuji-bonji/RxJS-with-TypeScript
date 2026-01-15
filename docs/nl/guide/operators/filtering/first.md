---
description: De first operator haalt alleen de eerste waarde uit de stream, of de eerste waarde die aan de gespecificeerde voorwaarde voldoet, en voltooit dan de stream. Dit is nuttig wanneer u alleen de eerste bereikte gebeurtenis wilt verwerken of initiële data wilt ophalen.
titleTemplate: ':title'
---

# first - Haal Eerste Waarde

De `first` operator haalt alleen de **eerste waarde** of **eerste waarde die aan een voorwaarde voldoet** uit een stream en voltooit de stream.


## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Haal alleen de eerste waarde
numbers$.pipe(
  first()
).subscribe(console.log);

// Haal alleen de eerste waarde die aan de voorwaarde voldoet
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// Output:
// 1
// 4
```

- `first()` haalt de eerste waarde die stroomt en voltooit.
- Als een voorwaarde wordt doorgegeven, wordt de **eerste waarde die aan de voorwaarde voldoet** opgehaald.
- Als er geen waarde bestaat die aan de voorwaarde voldoet, wordt een fout gegenereerd.

[🌐 RxJS Officiële Documentatie - `first`](https://rxjs.dev/api/operators/first)


## 💡 Typische gebruikspatronen

- Verwerk alleen de eerste bereikte gebeurtenis
- Detecteer de eerste data die aan de criteria voldoet (bijv. een score van 5 of hoger)
- Adopteer alleen de eerste data die binnenkwam voor een timeout of annulering


## 🧠 Praktisch codevoorbeeld (met UI)

Verwerk **alleen de eerste klik** zelfs als de knop meerdere keren wordt geklikt.

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>first Praktisch Voorbeeld:</h3>';
document.body.appendChild(title);

// Maak knop
const button = document.createElement('button');
button.textContent = 'Klik alstublieft (reageer alleen de eerste keer)';
document.body.appendChild(button);

// Maak uitvoergebied
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// Knopklik stream
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `Eerste klik gedetecteerd! ${count}`;
    output.appendChild(message);
  });
```

- Alleen de eerste klikgebeurtenis wordt ontvangen, en volgende gebeurtenissen worden genegeerd.
- De stream zal automatisch `complete` na de eerste klik.
