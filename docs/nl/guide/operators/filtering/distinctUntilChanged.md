---
description: De distinctUntilChanged operator maakt efficiënte dataverwerking mogelijk door opeenvolgende waarden die hetzelfde zijn als de vorige over te slaan en alleen de waarden die zijn veranderd uit te geven.
titleTemplate: ':title | RxJS'
---

# distinctUntilChanged - Geen Duplicaten

De `distinctUntilChanged` operator verwijdert duplicaten wanneer dezelfde waarde opeenvolgend wordt uitgegeven, en geeft alleen de nieuwe waarde uit als deze verschilt van de vorige waarde.


## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Output: 1, 2, 3, 1, 2, 3
```

- Als de waarde hetzelfde is als de vorige, wordt deze genegeerd.
- Dit is geen batchproces zoals `Array.prototype.filter`, maar eerder een **sequentiële beslissing**.

[🌐 RxJS Officiële Documentatie - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)


## 💡 Typische gebruikspatronen

- Formulierinvoerdetectie om verspilde verzoeken voor opeenvolgende dezelfde invoerwaarden te voorkomen
- Detecteren van wijzigingen in sensoren en gebeurtenisstreams
- Voorkom onnodige UI-hertekeningen in statusbeheer


## 🧠 Praktisch codevoorbeeld (met UI)

Simulatie van het verzenden van een API-verzoek in een zoekvak **alleen als de ingevoerde string verschilt van de vorige**.

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Maak uitvoergebied
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Voer zoekwoorden in';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Invoerstream
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `Uitvoeren met zoekwaarde: ${keyword}`;
  });

```

- Als de invoertekst niet verandert, wordt er geen verzoek gedaan.
- Dit kan worden gebruikt voor efficiënte zoekverwerking en API-communicatieoptimalisatie.
