---
description: mergeAll è un operatore che prende un Higher-order Observable (Observable di Observable) e sottoscrive tutti gli Observable interni in parallelo per appiattire i valori.
titleTemplate: ':title | RxJS'
---

# mergeAll - Appiattisci in Parallelo

L'operatore `mergeAll` prende un **Higher-order Observable** (Observable di Observable),
**sottoscrive tutti gli Observable interni in parallelo**, e appiattisce i loro valori.

## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent, interval } from 'rxjs';
import { map, mergeAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Avvia un nuovo contatore per ogni click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Sottoscrivi tutti i contatori in parallelo
higherOrder$
  .pipe(mergeAll())
  .subscribe(x => console.log(x));

// Output (con 3 click):
// 0 (1° contatore)
// 1 (1° contatore)
// 0 (2° contatore) ← esecuzione parallela
// 2 (1° contatore)
// 1 (2° contatore)
// 0 (3° contatore) ← esecuzione parallela
// ...
```

- **Sottoscrivi in parallelo** ogni Observable interno emesso dal Higher-order Observable
- **Combina i valori** da tutti gli Observable interni in un **singolo stream**
- Può limitare il numero di subscription concorrenti (`mergeAll(2)` = fino a 2 concorrenti)

[🌐 Documentazione Ufficiale RxJS - `mergeAll`](https://rxjs.dev/api/index/function/mergeAll)

## 💡 Pattern di Utilizzo Tipici

- **Eseguire più chiamate API in parallelo**
- **Avviare stream indipendenti per ogni azione utente**
- **Integrare più connessioni real-time come WebSocket e EventSource**

## 🧠 Esempio di Codice Pratico

Esempio di esecuzione di chiamate API concorrenti (simulate) ad ogni cambio di input

```ts
import { fromEvent, of } from 'rxjs';
import { map, mergeAll, delay, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Inserisci parole chiave di ricerca';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

// Debounce eventi input
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Chiamata API simulata per ogni valore di input
const results$ = search$.pipe(
  map(query =>
    // Chiamata API simulata (ritardo 500ms)
    of(`Risultato: "${query}"`).pipe(delay(500))
  ),
  mergeAll() // Esegui tutte le chiamate API in parallelo
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- **Tutte le chiamate API vengono eseguite in parallelo**, anche se l'utente cambia rapidamente l'input
- I vecchi risultati di ricerca possono apparire dopo i nuovi risultati (nessuna garanzia di ordine)

## 🔄 Operatori Correlati

| Operatore | Descrizione |
|---|---|
| `mergeMap` | Abbreviazione per `map` + `mergeAll` (comunemente usato) |
| [concatAll](/it/guide/operators/combination/concatAll) | Sottoscrive gli Observable interni in ordine (attendi completamento precedente) |
| [switchAll](/it/guide/operators/combination/switchAll) | Passa al nuovo Observable interno (annulla il vecchio) |
| [exhaustAll](/it/guide/operators/combination/exhaustAll) | Ignora nuovi Observable interni durante l'esecuzione |

## ⚠️ Note Importanti

### Limitare le Subscription Concorrenti

La mancata limitazione delle subscription concorrenti può causare problemi di performance.

```ts
// Limita le subscription concorrenti a 2
higherOrder$.pipe(
  mergeAll(2) // Fino a 2 esecuzioni concorrenti
).subscribe();
```

### Nessuna Garanzia di Ordine

Poiché `mergeAll` esegue concorrentemente, **l'ordine dei valori non è garantito**.
Se l'ordine è critico, usa [concatAll](/it/guide/operators/combination/concatAll).
