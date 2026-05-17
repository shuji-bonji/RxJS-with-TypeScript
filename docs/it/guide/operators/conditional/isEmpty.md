---
description: L'operatore isEmpty determina se Observable è completato senza emettere un valore e viene usato per rilevamento dati vuoti e ramificazione condizionale.
---

# isEmpty - Determina se lo Stream è Vuoto

L'operatore `isEmpty` **emette `true` se Observable si completa senza emettere alcun valore**.
Se emette anche un solo valore, emette `false` e si completa.

## 🔰 Sintassi e Operazione Base

```ts
import { of, EMPTY } from 'rxjs';
import { isEmpty } from 'rxjs';

EMPTY.pipe(isEmpty()).subscribe(console.log); // Output: true
of(1).pipe(isEmpty()).subscribe(console.log); // Output: false
```

[🌐 Documentazione Ufficiale RxJS - isEmpty](https://rxjs.dev/api/index/function/isEmpty)

## 💡 Esempi di Utilizzo Tipici

- Quando vuoi determinare se il risultato del filtraggio o della ricerca è vuoto
- Quando vuoi emettere un errore o passare a un altro processo se è vuoto

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

from([1, 3, 5])
  .pipe(
    filter((x) => x % 2 === 0),
    isEmpty()
  )
  .subscribe((result) => {
    console.log('È vuoto:', result);
  });

// Output:
// È vuoto: true
```

## 🧪 Esempi di Codice Pratici (con UI)

### ✅ 1. Determinare se il Risultato è Vuoto

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Esempio operatore isEmpty:</h3>';
document.body.appendChild(container);

const checkButton = document.createElement('button');
checkButton.textContent = 'Controlla se contiene numeri pari';
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
        ? 'Nessun numero pari trovato.'
        : 'Numeri pari inclusi.';
      output.style.color = isEmptyResult ? 'red' : 'green';
    });
});
```

### ✅ 2. Controllare se i Risultati di Ricerca Utente sono Vuoti

```ts
import { fromEvent, of, from } from 'rxjs';
import { debounceTime, switchMap, map, filter, isEmpty, delay } from 'rxjs';

const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Controllo risultati ricerca con isEmpty:</h3>';
document.body.appendChild(searchContainer);

const input = document.createElement('input');
input.placeholder = 'Inserisci parola di ricerca';
input.style.marginBottom = '10px';
searchContainer.appendChild(input);

const resultBox = document.createElement('div');
resultBox.style.padding = '10px';
resultBox.style.border = '1px solid #ccc';
searchContainer.appendChild(resultBox);

const mockData = ['mela', 'banana', 'arancia', 'uva'];

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
      ? 'Nessun elemento corrispondente trovato'
      : 'Elementi corrispondenti trovati';
    resultBox.style.color = noResults ? 'red' : 'green';
  });
```

> [!WARNING] Attenzione in codice di produzione
> L'esempio sopra omette la disiscrizione di `fromEvent` per semplificare la spiegazione. In codice reale, gestisci esplicitamente il ciclo di vita con `takeUntil(destroy$)`, `take(N)`, o `Subscription.unsubscribe()`. Dettagli: [Superare le difficoltà: gestione del ciclo di vita](/it/guide/overcoming-difficulties/lifecycle-management.md)
