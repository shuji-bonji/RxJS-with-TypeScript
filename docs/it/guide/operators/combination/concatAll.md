---
description: concatAll è un operatore che prende un Higher-order Observable (Observable di Observable) e appiattisce i valori sottoscrivendo gli Observable interni in ordine. Avvia il successivo dopo che l'Observable precedente completa.
titleTemplate: ':title | RxJS'
---

# concatAll - Appiattisci in Sequenza

L'operatore `concatAll` prende un **Higher-order Observable** (Observable di Observable),
**sottoscrive gli Observable interni in ordine**, e appiattisce i loro valori. Non avvierà il successivo fino a quando l'Observable precedente non completa.

## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent, interval } from 'rxjs';
import { map, concatAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Avvia un nuovo contatore per ogni click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Sottoscrivi i contatori in ordine (avvia il successivo dopo che il precedente completa)
higherOrder$
  .pipe(concatAll())
  .subscribe(x => console.log(x));

// Output (con 3 click):
// 0 (1° contatore)
// 1 (1° contatore)
// 2 (1° contatore) ← Completo
// 0 (2° contatore) ← Avvia dopo che il 1° completa
// 1 (2° contatore)
// 2 (2° contatore) ← Completo
// 0 (3° contatore) ← Avvia dopo che il 2° completa
// ...
```

- **Sottoscrivi in ordine** ogni Observable interno emesso dal Higher-order Observable
- **Non avviare il successivo** fino a quando l'Observable interno precedente non completa
- L'ordine dei valori è garantito

[🌐 Documentazione Ufficiale RxJS - `concatAll`](https://rxjs.dev/api/index/function/concatAll)

## 💡 Pattern di Utilizzo Tipici

- **Eseguire chiamate API in sequenza (esegui la successiva dopo che la richiesta precedente completa)**
- **Riprodurre animazioni in ordine**
- **Elaborare upload di file sequenzialmente**

## 🧠 Esempio di Codice Pratico

Esempio di esecuzione di chiamate API (simulate) in ordine per ogni click del pulsante

```ts
import { fromEvent, of } from 'rxjs';
import { map, concatAll, delay } from 'rxjs';

const button = document.createElement('button');
button.textContent = 'Chiamata API';
document.body.appendChild(button);

const output = document.createElement('div');
document.body.appendChild(output);

let callCount = 0;

// Evento click del pulsante
const clicks$ = fromEvent(button, 'click');

// Higher-order Observable: Chiamata API simulata per ogni click
const results$ = clicks$.pipe(
  map(() => {
    const id = ++callCount;
    const start = Date.now();

    // Chiamata API simulata (ritardo 2 secondi)
    return of(`Chiamata API #${id} completata`).pipe(
      delay(2000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} secondi)`;
      })
    );
  }),
  concatAll() // Esegui tutte le chiamate API in ordine
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Anche con click consecutivi del pulsante, **le chiamate API vengono eseguite in ordine**
- La chiamata API successiva inizia dopo che la precedente completa

## 🔄 Operatori Correlati

| Operatore | Descrizione |
|---|---|
| `concatMap` | Abbreviazione per `map` + `concatAll` (comunemente usato) |
| [mergeAll](/it/guide/operators/combination/mergeAll) | Sottoscrive tutti gli Observable interni in parallelo |
| [switchAll](/it/guide/operators/combination/switchAll) | Passa al nuovo Observable interno (annulla il vecchio) |
| [exhaustAll](/it/guide/operators/combination/exhaustAll) | Ignora nuovi Observable interni durante l'esecuzione |

## ⚠️ Note Importanti

### Backpressure (Accumulo Coda)

Se il tasso di emissione dell'Observable interno è più veloce del tasso di completamento, **gli Observable non elaborati si accumuleranno nella coda**.

```ts
// Click ogni secondo → Chiamata API impiega 2 secondi
// → La coda potrebbe accumularsi continuamente
```

In questo caso, considera queste contromisure:
- Usa `switchAll` (elabora solo l'ultimo)
- Usa `exhaustAll` (ignora durante l'esecuzione)
- Aggiungi debounce o throttling

### Attenzione agli Observable Infiniti

Se l'Observable precedente **non completa mai, il successivo non inizierà mai**.

#### ❌ interval non completa mai, quindi il 2° contatore non inizia mai
```ts
clicks$.pipe(
  map(() => interval(1000)), // Non completa mai
  concatAll()
).subscribe();
```
#### ✅ Completa con take
```ts
clicks$.pipe(
  map(() => interval(1000).pipe(take(3))), // Completa dopo 3
  concatAll()
).subscribe();
```
