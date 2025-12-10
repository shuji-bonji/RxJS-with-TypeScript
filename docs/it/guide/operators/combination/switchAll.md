---
description: switchAll è un operatore che prende un Higher-order Observable (Observable di Observable), passa a un nuovo Observable interno e annulla quello vecchio.
---

# switchAll - Passa al Nuovo Observable Interno

L'operatore `switchAll` prende un **Higher-order Observable** (Observable di Observable),
**passa ogni volta che viene emesso un nuovo Observable interno**, e annulla il vecchio Observable interno.

## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent, interval } from 'rxjs';
import { map, switchAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Avvia un nuovo contatore per ogni click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Passa al nuovo contatore (annulla il vecchio contatore)
higherOrder$
  .pipe(switchAll())
  .subscribe(x => console.log(x));

// Output (con 3 click):
// 0 (1° contatore)
// 1 (1° contatore)
// ← Click qui (1° annullato)
// 0 (2° contatore) ← Passa al nuovo contatore
// ← Click qui (2° annullato)
// 0 (3° contatore) ← Passa al nuovo contatore
// 1 (3° contatore)
// 2 (3° contatore)
```

- Quando viene emesso un nuovo Observable interno dal Higher-order Observable, **passa immediatamente**
- L'Observable interno precedente viene **automaticamente annullato**
- Solo l'ultimo Observable interno è sempre in esecuzione

[🌐 Documentazione Ufficiale RxJS - `switchAll`](https://rxjs.dev/api/index/function/switchAll)

## 💡 Pattern di Utilizzo Tipici

- **Funzionalità di ricerca (annulla vecchie ricerche ad ogni input)**
- **Autocomplete**
- **Aggiornamenti dati real-time (passa all'ultima sorgente dati)**

## 🧠 Esempio di Codice Pratico

Esempio di annullamento vecchie ricerche ed esecuzione solo dell'ultima ricerca ad ogni input

```ts
import { fromEvent, of } from 'rxjs';
import { map, switchAll, debounceTime, delay } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Inserisci parole chiave di ricerca';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

let searchCount = 0;

// Debounce eventi input
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Chiamata API di ricerca simulata per ogni valore di input
const results$ = search$.pipe(
  map(query => {
    const id = ++searchCount;
    const start = Date.now();

    // Chiamata API di ricerca simulata (ritardo 1 secondo)
    return of(`Risultati ricerca: "${query}"`).pipe(
      delay(1000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `[Ricerca #${id}] ${msg} (${elapsed} secondi)`;
      })
    );
  }),
  switchAll() // Annulla vecchia ricerca quando ne inizia una nuova
);

results$.subscribe(result => {
  output.innerHTML = ''; // Pulisci risultati precedenti
  const item = document.createElement('div');
  item.textContent = result;
  output.appendChild(item);
});
```

- **Le vecchie ricerche vengono automaticamente annullate** quando l'utente cambia input
- Solo gli ultimi risultati di ricerca vengono sempre visualizzati

## 🔄 Operatori Correlati

| Operatore | Descrizione |
|---|---|
| `switchMap` | Abbreviazione per `map` + `switchAll` (più comunemente usato) |
| [mergeAll](/it/guide/operators/combination/mergeAll) | Sottoscrive tutti gli Observable interni in parallelo |
| [concatAll](/it/guide/operators/combination/concatAll) | Sottoscrive gli Observable interni in ordine (attendi completamento precedente) |
| [exhaustAll](/it/guide/operators/combination/exhaustAll) | Ignora nuovi Observable interni durante l'esecuzione |

## ⚠️ Note Importanti

### Prevenzione Perdite di Memoria

`switchAll` aiuta a prevenire perdite di memoria **annullando automaticamente** i vecchi Observable interni.
È ideale per nuove richieste frequenti come ricerche o autocomplete.

### Observable Interni Non Completanti

Anche se l'Observable interno non completa, passerà automaticamente quando viene emesso un nuovo Observable interno.

```ts
// interval non completa mai, ma viene automaticamente annullato al prossimo click
clicks$.pipe(
  map(() => interval(1000)), // Non completa mai
  switchAll()
).subscribe();
```

### Ottimale Quando Solo l'Ultimo Valore Conta

Usa `switchAll` quando non hai bisogno dei risultati delle vecchie elaborazioni e **solo l'ultimo risultato è importante**.
Se tutti i risultati sono necessari, usa [mergeAll](/it/guide/operators/combination/mergeAll).
