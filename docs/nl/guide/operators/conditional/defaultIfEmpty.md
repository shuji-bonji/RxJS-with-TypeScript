---
description: "De defaultIfEmpty operator retourneert een standaardwaarde wanneer een Observable geen waarden uitzendt. Behandelt lege API-responsen, initiële waarde-supplementatie en fallbacks voor zoekresultaten zonder resultaten. Bevat praktische use cases en type-veilige implementatie in TypeScript."
---

# defaultIfEmpty - Standaardwaarde voor Lege Streams

De `defaultIfEmpty` operator is een operator die **een opgegeven standaardwaarde uitzendt wanneer een Observable zonder waarden uit te zenden completeert**.
Het wordt gebruikt om te reageren op lege arrays of lege API-resultaten.

## 🔰 Basissyntax en Werking

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

from([]).pipe(
  defaultIfEmpty('Geen waarden')
).subscribe(console.log);

// Output:
// Geen waarden
```

In dit voorbeeld wordt voor een lege array die met `from` tot Observable wordt gemaakt, `'Geen waarden'` uitgevoerd via `defaultIfEmpty`.

[🌐 RxJS Officiële Documentatie - defaultIfEmpty](https://rxjs.dev/api/index/function/defaultIfEmpty)

## 💡 Typische Gebruiksvoorbeelden

- Wanneer een gebruiker niets heeft ingevoerd
- Wanneer een API een leeg resultaat retourneert
- Wanneer er geen enkele waarde is die aan een voorwaarde voldoet

wordt het gebruikt om **de situatie "niets werd geretourneerd" aan te vullen**.

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of(['A', 'B', 'C']).pipe(delay(500))
    : EMPTY.pipe(delay(500));
}

mockApiCall(false)
  .pipe(defaultIfEmpty('Geen gegevens'))
  .subscribe(console.log);

// Output:
// Geen gegevens
```

## 🧪 Praktische Code Voorbeelden (met UI)

### ✅ 1. Controleren op Lege Arrays

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

// UI opbouwen
const container = document.createElement('div');
container.innerHTML = '<h3>defaultIfEmpty operator voorbeeld:</h3>';
document.body.appendChild(container);

const emptyBtn = document.createElement('button');
emptyBtn.textContent = 'Verwerk lege array';
container.appendChild(emptyBtn);

const nonEmptyBtn = document.createElement('button');
nonEmptyBtn.textContent = 'Verwerk niet-lege array';
container.appendChild(nonEmptyBtn);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

emptyBtn.addEventListener('click', () => {
  result.textContent = 'Verwerken...';
  from([]).pipe(
    defaultIfEmpty('Geen gegevens')
  ).subscribe(value => {
    result.textContent = `Resultaat: ${value}`;
  });
});

nonEmptyBtn.addEventListener('click', () => {
  result.textContent = 'Verwerken...';
  from([1, 2, 3]).pipe(
    defaultIfEmpty('Geen gegevens')
  ).subscribe(value => {
    result.textContent = `Resultaat: ${value}`;
  });
});
```

### ✅ 2. Standaardwaarden Invullen voor Lege API-resultaten

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of([
        { id: 1, name: 'Item 1' },
        { id: 2, name: 'Item 2' },
      ]).pipe(delay(1000))
    : EMPTY.pipe(delay(1000));
}

const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>API-resultaat verwerking met defaultIfEmpty:</h3>';
document.body.appendChild(apiContainer);

const dataBtn = document.createElement('button');
dataBtn.textContent = 'Met gegevens';
dataBtn.style.marginRight = '10px';
apiContainer.appendChild(dataBtn);

const emptyBtn2 = document.createElement('button');
emptyBtn2.textContent = 'Zonder gegevens';
apiContainer.appendChild(emptyBtn2);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
apiContainer.appendChild(output);

dataBtn.addEventListener('click', () => {
  output.textContent = 'Ophalen...';
  mockApiCall(true)
    .pipe(defaultIfEmpty('Geen gegevens gevonden'))
    .subscribe({
      next: (val) => {
        if (Array.isArray(val)) {
          const ul = document.createElement('ul');
          val.forEach((item) => {
            const li = document.createElement('li');
            li.textContent = `${item.id}: ${item.name}`;
            ul.appendChild(li);
          });
          output.innerHTML = '<h4>Opgehaalde resultaten:</h4>';
          output.appendChild(ul);
        } else {
          output.textContent = val;
        }
      },
    });
});

emptyBtn2.addEventListener('click', () => {
  output.textContent = 'Ophalen...';
  mockApiCall(false)
    .pipe(defaultIfEmpty('Geen gegevens gevonden'))
    .subscribe({
      next: (val) => {
        output.textContent = val.toString();
      },
    });
});

```
