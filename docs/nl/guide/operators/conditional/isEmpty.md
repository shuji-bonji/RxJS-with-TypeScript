---
description: "De isEmpty operator bepaalt of een Observable is voltooid zonder waarden uit te zenden. Wordt gebruikt voor lege gegevensdetectie, voorwaardelijke vertakkingen en gegevensbestaanscontroles. Inclusief detectie van lege resultaten na filter(), en type-veilige implementatie in TypeScript uitgelegd met praktische code voorbeelden."
---

# isEmpty - Bepalen of een Stream Leeg is

De `isEmpty` operator **zendt `true` uit wanneer een Observable voltooit zonder een enkele waarde uit te zenden**.
Als zelfs één waarde wordt uitgezonden, zendt het `false` uit en voltooit het.

## 🔰 Basissyntax en Werking

```ts
import { of, EMPTY } from 'rxjs';
import { isEmpty } from 'rxjs';

EMPTY.pipe(isEmpty()).subscribe(console.log); // Output: true
of(1).pipe(isEmpty()).subscribe(console.log); // Output: false
```

[🌐 RxJS Officiële Documentatie - isEmpty](https://rxjs.dev/api/index/function/isEmpty)

## 💡 Typische Gebruiksvoorbeelden

- Wanneer u wilt bepalen of filterresultaten of zoekresultaten leeg zijn
- Wanneer u een fout wilt genereren of wilt overschakelen naar een andere verwerking als het leeg is

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

from([1, 3, 5])
  .pipe(
    filter((x) => x % 2 === 0),
    isEmpty()
  )
  .subscribe((result) => {
    console.log('Leeg of niet:', result);
  });

// Output:
// Leeg of niet: true
```

## 🧪 Praktische Code Voorbeelden (met UI)

### ✅ 1. Bepalen of een Resultaat Leeg is

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>isEmpty operator voorbeeld:</h3>';
document.body.appendChild(container);

const checkButton = document.createElement('button');
checkButton.textContent = 'Controleer of even getallen voorkomen';
container.appendChild(checkButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
container.appendChild(output);

checkButton.addEventListener('click', () => {
  from([1, 3, 5])
    .pipe(
      filter((x) => x % 2 === 0),
      isEmpty()
    )
    .subscribe((isEmptyResult) => {
      output.textContent = isEmptyResult
        ? 'Er waren geen even getallen.'
        : 'Er zijn even getallen aanwezig.';
      output.style.color = isEmptyResult ? 'red' : 'green';
    });
});
```

### ✅ 2. Controleren of Gebruikerszoekresultaten Leeg zijn

```ts
import { fromEvent, of, from } from 'rxjs';
import { debounceTime, switchMap, map, filter, isEmpty, delay } from 'rxjs';

const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Zoekresultaat controle met isEmpty:</h3>';
document.body.appendChild(searchContainer);

const input = document.createElement('input');
input.placeholder = 'Voer zoekterm in';
input.style.marginBottom = '10px';
searchContainer.appendChild(input);

const resultBox = document.createElement('div');
resultBox.style.padding = '10px';
resultBox.style.border = '1px solid #ccc';
searchContainer.appendChild(resultBox);

const mockData = ['apple', 'banana', 'orange', 'grape'];

fromEvent(input, 'input')
  .pipe(
    debounceTime(300),
    map((e) => (e.target as HTMLInputElement).value.trim().toLowerCase()),
    filter((text) => text.length > 0),
    switchMap((query) =>
      of(mockData).pipe(
        delay(300),
        map((list) => list.filter((item) => item.includes(query))),
        switchMap((filtered) => from(filtered).pipe(isEmpty()))
      )
    )
  )
  .subscribe((noResults) => {
    resultBox.textContent = noResults
      ? 'Geen overeenkomende items gevonden'
      : 'Overeenkomende items gevonden';
    resultBox.style.color = noResults ? 'red' : 'green';
  });
```
