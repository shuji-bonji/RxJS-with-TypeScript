---
description: De exhaustMap operator is een conversieoperator die nieuwe invoer negeert tot de momenteel verwerkende Observable is voltooid. Het is effectief in situaties waar u gelijktijdigheid wilt beperken, zoals het voorkomen van meerdere klikken op een formulier submit-knop of dubbele API-verzoekinzendingen.
---

# exhaustMap - Negeer nieuwe invoer tijdens uitvoering

De `exhaustMap` operator **negeert nieuwe invoer** tot de momenteel verwerkende Observable is voltooid.
Dit is ideaal voor het voorkomen van dubbele klikken of meerdere inzendingen van verzoeken.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('Verzoek voltooid').pipe(delay(1000))))
  .subscribe(console.log);

// Output voorbeeld:
// (Alleen de eerste klik geeft "Verzoek voltooid" uit na 1 seconde)

```

- Volgende invoer wordt genegeerd tot het lopende verzoek is voltooid.

[🌐 RxJS Officiële Documentatie - `exhaustMap`](https://rxjs.dev/api/operators/exhaustMap)

## 💡 Typische gebruikspatronen

- Preventie van meerdere klikken op formulier submit-knoppen
- Preventie van dubbele verzoeken (vooral voor login- en betalingsprocessen)
- Enkelvoudige weergavecontrole van een modal of dialoog

## 🧠 Praktisch codevoorbeeld (met UI)

Klikken op de Verzend-knop start het verzendproces.
**Het maakt niet uit hoe vaak u klikt tijdens verzending, het wordt genegeerd** en de volgende verzending wordt niet geaccepteerd tot het eerste proces is voltooid.

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Maak knop
const submitButton = document.createElement('button');
submitButton.textContent = 'Verzenden';
document.body.appendChild(submitButton);

// Maak uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Verzendverwerking
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = 'Verzenden...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // Simuleer 2 seconden verzendvertraging
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = 'Verzending succesvol!';
      console.log('Verzending succesvol:', response);
    },
    error: (error) => {
      output.textContent = 'Verzendfout';
      console.error('Verzendfout:', error);
    },
  });

```

- Andere klikken terwijl de knop wordt geklikt worden genegeerd.
- Na 2 seconden ziet u "Verzending succesvol!" of "Verzendfout" wordt weergegeven.
