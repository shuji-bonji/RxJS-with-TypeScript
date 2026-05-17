---
description: "L'operatore filter seleziona i valori in uno stream in base a una funzione condizionale e fa passare solo quelli che soddisfano la condizione. Efficiente per validazione input form, estrazione dati con condizioni specifiche, esclusione di null/undefined. Utilizzabile anche come type guard TypeScript."
---

# filter - Filtra per Condizione

L'operatore `filter` seleziona i valori in uno stream in base a una funzione condizionale specificata e fa passare solo i valori che soddisfano la condizione.

## 🔰 Sintassi di base e utilizzo

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Output: 2, 4, 6, 8, 10
```

- Passano solo i valori che soddisfano la condizione.
- Simile a `Array.prototype.filter()`, ma elaborato sequenzialmente su Observable.

[🌐 Documentazione ufficiale RxJS - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 Pattern di utilizzo tipici

- Validazione dei valori di input dei form
- Permettere solo dati con tipo o struttura specifica
- Filtering di eventi sensore o dati stream

## 🧠 Esempio di codice pratico (con UI)

Visualizza in tempo reale in una lista solo quando il numero inserito è pari.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'Esempio pratico di filter:';
document.body.appendChild(title);

// Crea campo di input
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Inserisci numero';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Crea area di output
const output = document.createElement('div');
document.body.appendChild(output);

// Stream di eventi di input
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Rilevato numero pari: ${evenNumber}`;
    output.prepend(item);
  });

```

- Solo quando il numero è pari, viene visualizzato nell'output.
- Numeri dispari o input non validi vengono ignorati.

> [!WARNING] Attenzione in codice di produzione
> L'esempio sopra omette la disiscrizione di `fromEvent` per semplificare la spiegazione. In codice reale, gestisci esplicitamente il ciclo di vita con `takeUntil(destroy$)`, `take(N)`, o `Subscription.unsubscribe()`. Dettagli: [Superare le difficoltà: gestione del ciclo di vita](/it/guide/overcoming-difficulties/lifecycle-management.md)
