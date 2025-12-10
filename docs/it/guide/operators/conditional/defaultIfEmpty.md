---
description: L'operatore defaultIfEmpty è un operatore per restituire un valore predefinito se Observable non emette un valore, utile per gestione dati vuoti e completamento valore iniziale.
titleTemplate: ':title | RxJS'
---

# defaultIfEmpty - Valore Predefinito se lo Stream è Vuoto

L'operatore `defaultIfEmpty` è un **operatore che emette un valore predefinito specificato se Observable viene completato senza emettere alcun valore**.
Viene usato per gestire array vuoti o risultati API vuoti.

## 🔰 Sintassi e Operazione Base

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

from([]).pipe(
  defaultIfEmpty('Nessun valore')
).subscribe(console.log);

// Output:
// Nessun valore
```

In questo esempio, `defaultIfEmpty` emetterà `'Nessun valore'` per un array vuoto reso Observable con `from`.

[🌐 Documentazione Ufficiale RxJS - defaultIfEmpty](https://rxjs.dev/api/index/function/defaultIfEmpty)

## 💡 Esempi di Utilizzo Tipici

- Se l'utente non ha inserito alcuna informazione
- Quando l'API restituisce un risultato vuoto
- Se nessuno dei valori soddisfa le condizioni

Questo viene usato per **completare la situazione "nulla restituito"** in casi come questi.

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of(['A', 'B', 'C']).pipe(delay(500))
    : EMPTY.pipe(delay(500));
}

mockApiCall(false)
  .pipe(defaultIfEmpty('Nessun dato'))
  .subscribe(console.log);

// Output:
// Nessun dato
```

## 🧪 Esempi di Codice Pratici (con UI)

### ✅ 1. Usato per Determinare se un Array è Vuoto

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

// Costruisci UI
const container = document.createElement('div');
container.innerHTML = '<h3>Esempio operatore defaultIfEmpty:</h3>';
document.body.appendChild(container);

const emptyBtn = document.createElement('button');
emptyBtn.textContent = 'Elabora array vuoto';
container.appendChild(emptyBtn);

const nonEmptyBtn = document.createElement('button');
nonEmptyBtn.textContent = 'Elabora array non vuoto';
container.appendChild(nonEmptyBtn);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

emptyBtn.addEventListener('click', () => {
  result.textContent = 'Elaborazione...';
  from([]).pipe(
    defaultIfEmpty('Nessun dato')
  ).subscribe(value => {
    result.textContent = `Risultato: ${value}`;
  });
});

nonEmptyBtn.addEventListener('click', () => {
  result.textContent = 'Elaborazione...';
  from([1, 2, 3]).pipe(
    defaultIfEmpty('Nessun dato')
  ).subscribe(value => {
    result.textContent = `Risultato: ${value}`;
  });
});
```

### ✅ 2. Completare Default per Risultato Vuoto nell'API

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of([
        { id: 1, name: 'Elemento 1' },
        { id: 2, name: 'Elemento 2' },
      ]).pipe(delay(1000))
    : EMPTY.pipe(delay(1000));
}

const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Elaborazione risultato API con defaultIfEmpty:</h3>';
document.body.appendChild(apiContainer);

const dataBtn = document.createElement('button');
dataBtn.textContent = 'Con dati';
dataBtn.style.marginRight = '10px';
apiContainer.appendChild(dataBtn);

const emptyBtn2 = document.createElement('button');
emptyBtn2.textContent = 'Senza dati';
apiContainer.appendChild(emptyBtn2);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
apiContainer.appendChild(output);

dataBtn.addEventListener('click', () => {
  output.textContent = 'Recupero...';
  mockApiCall(true)
    .pipe(defaultIfEmpty('Nessun dato trovato'))
    .subscribe({
      next: (val) => {
        if (Array.isArray(val)) {
          const ul = document.createElement('ul');
          val.forEach((item) => {
            const li = document.createElement('li');
            li.textContent = `${item.id}: ${item.name}`;
            ul.appendChild(li);
          });
          output.innerHTML = '<h4>Risultato:</h4>';
          output.appendChild(ul);
        } else {
          output.textContent = val;
        }
      },
    });
});

emptyBtn2.addEventListener('click', () => {
  output.textContent = 'Recupero...';
  mockApiCall(false)
    .pipe(defaultIfEmpty('Nessun dato trovato'))
    .subscribe({
      next: (val) => {
        output.textContent = val.toString();
      },
    });
});
```
