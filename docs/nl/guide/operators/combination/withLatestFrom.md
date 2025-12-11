---
description: "withLatestFrom operator combineert de laatste waarde van een andere stream elke keer dat de hoofd-Observable emitteert: Ideaal voor formuliervalidatie en statussynchronisatie"
titleTemplate: ':title'
---

# withLatestFrom - Combineer laatste waarde bij emissie van hoofdstream

De `withLatestFrom` operator **elke keer dat een waarde in de hoofdstream wordt geëmitteerd**,
combineert de **laatste waarde** van een andere stream en geeft deze uit.


## 🔰 Basissyntax en gebruik

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

clicks$
  .pipe(
    withLatestFrom(timer$),
    map(([click, timerValue]) => `Teller op kliktijd: ${timerValue}`)
  )
  .subscribe(console.log);

// Output:
// Teller op kliktijd: 1
// Teller op kliktijd: 2
// Teller op kliktijd: 2
// Teller op kliktijd: 5

```

- De hoofd-Observable (in dit geval, klikken) fungeert als trigger,
- De **laatste waarde** van de sub-Observable (in dit geval, teller) wordt gecombineerd en elke keer uitgegeven.

[🌐 RxJS Officiële Documentatie - `withLatestFrom`](https://rxjs.dev/api/index/function/withLatestFrom)


## 💡 Typische gebruikspatronen

- **Laatste status ophalen bij gebruikersactie**
- **Gecachete data raadplegen bij verzoektijd**
- **Event-getriggerde databinding**


## 🧠 Praktisch codevoorbeeld (met UI)

Voorbeeld van het ophalen en weergeven van de laatste waarde van een invoerveld elke 2 seconden.

```ts
import { fromEvent, interval } from 'rxjs';
import { map, startWith, withLatestFrom } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'withLatestFrom: Haal Laatste Invoer Elke 2 Seconden:';
document.body.appendChild(title);

// Maak invoerveld
const nameInput = document.createElement('input');
nameInput.placeholder = 'Voer naam in';
document.body.appendChild(nameInput);

// Maak uitvoergebied
const output = document.createElement('div');
document.body.appendChild(output);

// Invoer Observable
const name$ = fromEvent(nameInput, 'input').pipe(
  map((e) => (e.target as HTMLInputElement).value),
  startWith('') // Begin met lege string
);

// Timer (vuurt elke 2 seconden)
const timer$ = interval(2000);

// Haal laatste invoerwaarde elke keer dat timer vuurt
timer$.pipe(withLatestFrom(name$)).subscribe(([_, name]) => {
  const item = document.createElement('div');
  item.textContent = `2-seconden ophalen: Naam: ${name}`;
  output.prepend(item);
});

```

- Terwijl de gebruiker doorgaat met typen,
- Wordt **de laatste invoer opgehaald en weergegeven** elke 2 seconden.
