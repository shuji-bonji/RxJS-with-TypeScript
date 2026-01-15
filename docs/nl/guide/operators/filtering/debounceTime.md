---
description: De debounceTime operator geeft de laatste waarde uit wanneer er geen nieuwe waarde is ontvangen gedurende een gespecificeerde tijd na het uitgeven van opeenvolgende gebeurtenissen. Dit is ideaal voor het optimaliseren van frequente invoer zoals zoekvak typen of venstergrootte wijzigen gebeurtenissen.
titleTemplate: ':title'
---

# debounceTime - Laatste waarde na stilte

De `debounceTime` operator geeft de laatste waarde uit nadat een waarde is uitgegeven in de stream als er geen nieuwe waarde is uitgegeven gedurende de gespecificeerde tijd.
Het wordt zeer vaak gebruikt in situaties waar frequente gebeurtenissen moeten worden onderdrukt, zoals zoekvakken met gebruikersinvoer.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- Als er geen verdere invoer wordt ontvangen binnen 300ms nadat een invoergebeurtenis optreedt, wordt de waarde uitgegeven.
- Dit heeft het effect van het consolideren van gebeurtenissen die kort achter elkaar optreden.

[🌐 RxJS Officiële Documentatie - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)

## 💡 Typische gebruikspatronen

- Verzend verzoek nadat gebruiker klaar is met typen in zoekvak
- Haal uiteindelijke grootte op voor venstergrootte wijzigen gebeurtenis
- Verkrijg uiteindelijke positie voor scrollgebeurtenis

## 🧠 Praktisch codevoorbeeld (met UI)

Wanneer een teken in het zoekvak wordt ingevoerd, wordt een bericht weergegeven dat zoeken start wanneer de invoer stopt voor 300 ms.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

// Maak uitvoergebied
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Voer zoekwoord in';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Invoerstream
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `Begonnen met zoeken naar "${value}"`;
});
```

- Geen onmiddellijke respons tijdens invoeren
- Het zal stoppen met invoeren en beginnen met zoeken met de laatste invoerwaarde 300ms later
