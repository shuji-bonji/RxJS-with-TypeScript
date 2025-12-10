---
description: mergeScan è un operatore RxJS che combina le caratteristiche di scan e mergeMap. Gestisce il valore precedente di aggregazione (accumulatore) mentre elabora Observable interni in parallelo. È ideale per aggregazione asincrona sequenziale e gestione stato con chiamate API.
titleTemplate: ':title | RxJS'
---

# mergeScan - Elaborazione di Aggregazione Asincrona

L'operatore `mergeScan` **combina le caratteristiche di `scan` e `mergeMap`, elaborando Observable interni in parallelo mentre mantiene il valore di aggregazione (accumulatore)**. È adatto per situazioni dove l'aggregazione richiede elaborazione asincrona.

## 🔰 Sintassi e Utilizzo Base

```ts
import { of } from 'rxjs';
import { mergeScan, delay } from 'rxjs';

// Aggregazione asincrona (aggiunge valore corrente al valore cumulativo precedente)
of(1, 2, 3, 4, 5)
  .pipe(
    mergeScan(
      (acc, value) => of(acc + value).pipe(delay(100)), // Aggregazione con elaborazione asincrona
      0 // Valore iniziale
    )
  )
  .subscribe(console.log);

// Output:
// 1   (0 + 1)
// 3   (1 + 2)
// 6   (3 + 3)
// 10  (6 + 4)
// 15  (10 + 5)
```

- `acc` è il valore cumulativo, `value` è il valore corrente.
- La funzione di proiezione deve restituire un `Observable`.
- Per ogni valore emesso, il risultato viene emesso accumulato nel risultato precedente.

[🌐 Documentazione Ufficiale RxJS - `mergeScan`](https://rxjs.dev/api/operators/mergeScan)

## 💡 Pattern di Utilizzo Tipici

- Aggregazione dati che richiede chiamate API
- Gestione stato che richiede elaborazione asincrona
- Elaborazione sequenziale mantenendo lo stato
- Caricamento dati impaginati con stato cumulativo

## 🔍 Differenza da scan e mergeMap

| Operatore | Funzione | Caso d'Uso |
|:---|:---|:---|
| `scan` | Aggregazione sincrona | Aggregazione semplice di numeri, gestione stato |
| `mergeMap` | Elaborazione asincrona (senza aggregazione) | Elaborazione parallela di Observable interni |
| `mergeScan` | **Aggregazione asincrona** | Aggregazione con chiamate API, aggregazione con stato |

```ts
import { of, timer } from 'rxjs';
import { scan, mergeMap, mergeScan, map } from 'rxjs';

const source$ = of(1, 2, 3);

// scan: Aggregazione sincrona
source$.pipe(scan((acc, val) => acc + val, 0)).subscribe(console.log);
// Output: 1, 3, 6

// mergeMap: Elaborazione asincrona (nessuna aggregazione)
source$
  .pipe(mergeMap(val => timer(100).pipe(map(() => val * 10))))
  .subscribe(console.log);
// Output: 10, 20, 30 (in parallelo, quindi l'ordine può variare)

// mergeScan: Aggregazione asincrona
source$
  .pipe(mergeScan((acc, val) => timer(100).pipe(map(() => acc + val)), 0))
  .subscribe(console.log);
// Output: 1, 3, 6 (elaborazione sequenziale con aggregazione)
```

## 🧠 Esempio di Codice Pratico 1: Aggregazione con Chiamate API

Questo esempio recupera dati da API e li aggrega.

```ts
import { from, Observable, of } from 'rxjs';
import { mergeScan, delay, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  score: number;
}

// API fittizia (ottieni dati utente)
function fetchUser(id: number): Observable<User> {
  return of({
    id,
    name: `Utente ${id}`,
    score: Math.floor(Math.random() * 100),
  }).pipe(delay(200)); // Simula ritardo di rete
}

// ID utente da recuperare
const userIds$ = from([1, 2, 3, 4, 5]);

interface AggregatedState {
  users: User[];
  totalScore: number;
}

// Recupera dati utente e aggrega
userIds$
  .pipe(
    mergeScan(
      (state, userId) =>
        fetchUser(userId).pipe(
          map(user => ({
            users: [...state.users, user],
            totalScore: state.totalScore + user.score,
          }))
        ),
      { users: [], totalScore: 0 } as AggregatedState
    )
  )
  .subscribe(state => {
    console.log(`Utenti: ${state.users.length}, Punteggio totale: ${state.totalScore}`);
  });

// Output:
// Utenti: 1, Punteggio totale: 42
// Utenti: 2, Punteggio totale: 89
// Utenti: 3, Punteggio totale: 134
// Utenti: 4, Punteggio totale: 178
// Utenti: 5, Punteggio totale: 225
```

## 🎯 Esempio di Codice Pratico 2: Caricamento Dati Impaginati

Questo esempio carica dati impaginati mantenendo la posizione corrente e i dati cumulativi.

```ts
import { Subject, Observable, of, EMPTY } from 'rxjs';
import { mergeScan, delay, map, tap, takeWhile } from 'rxjs';

interface PageData {
  items: string[];
  hasMore: boolean;
  nextPage: number;
}

interface LoadState {
  allItems: string[];
  currentPage: number;
  isComplete: boolean;
}

// API fittizia (ottieni dati pagina)
function fetchPage(page: number): Observable<PageData> {
  const totalPages = 5;
  return of({
    items: [`Pagina ${page} Elemento 1`, `Pagina ${page} Elemento 2`, `Pagina ${page} Elemento 3`],
    hasMore: page < totalPages,
    nextPage: page + 1,
  }).pipe(delay(300));
}

// Crea elementi UI
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = 'Carica Altro';
container.appendChild(loadButton);

const itemList = document.createElement('ul');
container.appendChild(itemList);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

// Trigger di caricamento
const loadMore$ = new Subject<void>();

// Stato
let currentState: LoadState = {
  allItems: [],
  currentPage: 1,
  isComplete: false,
};

loadMore$
  .pipe(
    mergeScan(
      (state, _) => {
        if (state.isComplete) {
          return EMPTY;
        }
        return fetchPage(state.currentPage).pipe(
          map(pageData => ({
            allItems: [...state.allItems, ...pageData.items],
            currentPage: pageData.nextPage,
            isComplete: !pageData.hasMore,
          }))
        );
      },
      currentState
    ),
    tap(state => {
      currentState = state;
    })
  )
  .subscribe(state => {
    // Aggiorna lista
    itemList.innerHTML = state.allItems.map(item => `<li>${item}</li>`).join('');

    // Aggiorna stato
    status.textContent = state.isComplete
      ? `Caricamento completato (${state.allItems.length} elementi)`
      : `Caricati ${state.allItems.length} elementi`;

    // Aggiorna bottone
    loadButton.disabled = state.isComplete;
    loadButton.textContent = state.isComplete ? 'Tutto caricato' : 'Carica Altro';
  });

// Gestione click bottone
loadButton.addEventListener('click', () => {
  loadMore$.next();
});

// Caricamento iniziale
loadMore$.next();
```

## 🎯 Controllo della Concorrenza

L'operatore `mergeScan` può controllare il numero di elaborazioni parallele con il terzo argomento `concurrent`.

```ts
import { interval, timer } from 'rxjs';
import { mergeScan, take, map } from 'rxjs';

interval(100)
  .pipe(
    take(10),
    mergeScan(
      (acc, value) =>
        timer(300).pipe(
          map(() => {
            console.log(`Elaborazione valore: ${value}, acc: ${acc}`);
            return acc + value;
          })
        ),
      0,
      2 // Massimo 2 elaborazioni parallele
    )
  )
  .subscribe(result => {
    console.log('Risultato:', result);
  });
```

**Casi d'uso del controllo concorrenza**:
- Prevenire sovraccarico delle API
- Gestire le limitazioni di risorse
- Controllo della priorità di elaborazione

## ⚠️ Note

### 1. Bisogna Restituire un Observable

La funzione di proiezione di `mergeScan` deve restituire un Observable.

```ts
// ❌ Esempio sbagliato: Restituisce valore non Observable
source$.pipe(
  mergeScan((acc, val) => acc + val, 0) // Errore di tipo!
);

// ✅ Esempio corretto: Wrap in Observable
source$.pipe(
  mergeScan((acc, val) => of(acc + val), 0)
);
```

### 2. Attenzione all'Ordine di Elaborazione

In elaborazione parallela, l'ordine di completamento può differire dall'ordine di emissione.

```ts
import { of, timer } from 'rxjs';
import { mergeScan, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    mergeScan(
      (acc, val) =>
        timer(Math.random() * 100).pipe(
          // Ritardo casuale
          map(() => {
            console.log(`Elaborato: ${val}`);
            return acc + val;
          })
        ),
      0
    )
  )
  .subscribe();
// L'ordine di elaborazione può cambiare

// Se l'ordine deve essere preservato, imposta concurrent a 1
of(1, 2, 3)
  .pipe(
    mergeScan(
      (acc, val) => timer(Math.random() * 100).pipe(map(() => acc + val)),
      0,
      1 // Elaborazione sequenziale
    )
  )
  .subscribe();
```

### 3. Gestione Errori

Se un errore si verifica nell'Observable interno, l'intero stream viene interrotto.

```ts
import { of, catchError, EMPTY } from 'rxjs';
import { mergeScan } from 'rxjs';

source$
  .pipe(
    mergeScan(
      (acc, val) =>
        fetchData(val).pipe(
          catchError(err => {
            console.error('Errore:', err);
            return of(acc); // Restituisci valore precedente in caso di errore
          })
        ),
      initialValue
    )
  )
  .subscribe();
```

## 📚 Operatori Correlati

- [`scan`](/it/guide/operators/transformation/scan) - Aggregazione sincrona
- [`mergeMap`](/it/guide/operators/transformation/mergeMap) - Elaborazione parallela di Observable interni
- [`concatMap`](/it/guide/operators/transformation/concatMap) - Elaborazione sequenziale di Observable interni
- [`expand`](/it/guide/operators/transformation/expand) - Elaborazione ricorsiva di Observable

## Riepilogo

L'operatore `mergeScan` è un operatore per **aggregazione asincrona**. Combina le caratteristiche di `scan` e `mergeMap`, ed è adatto per situazioni dove l'aggregazione richiede chiamate API o altra elaborazione asincrona. Con il controllo della concorrenza, puoi gestire l'elaborazione parallela di Observable interni. Tuttavia, nota che l'ordine di elaborazione può cambiare in elaborazione parallela. Se l'ordine deve essere preservato, imposta `concurrent` a `1`.
