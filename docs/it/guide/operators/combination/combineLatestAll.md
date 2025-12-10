---
description: combineLatestAll prende un Higher-order Observable (Observable di Observable) e combina l'ultimo valore di ciascuno quando tutti gli Observable interni hanno emesso almeno una volta.
titleTemplate: ':title'
---

# combineLatestAll - Combina gli Ultimi Valori di Tutti gli Observable Interni

L'operatore `combineLatestAll` prende un **Higher-order Observable** (Observable di Observable),
**una volta che tutti gli Observable interni hanno emesso almeno una volta**, combina i loro **ultimi valori** e li emette come array.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// Higher-order Observable con tre Observable interni
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Combina gli ultimi valori una volta che tutti gli Observable interni hanno emesso almeno una volta
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Output:
// [1, 3, 0] ← Quando tutti hanno emesso almeno una volta (dopo 2 secondi)
// [2, 3, 0] ← 1° Observable emette 2 (dopo 3 secondi)
// [2, 3, 1] ← 3° Observable emette 1 (dopo 4 secondi)
```

- Raccoglie gli Observable interni quando il Higher-order Observable **completa**
- **Una volta che tutti gli Observable interni hanno emesso almeno una volta**, inizia a combinare
- Ogni volta che un Observable interno emette un valore, **combina tutti gli ultimi valori** e li emette

[🌐 Documentazione Ufficiale RxJS - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Pattern di Utilizzo Tipici

- **Combinare gli ultimi risultati di più chiamate API**
- **Sincronizzare gli ultimi valori di più input di form**
- **Integrare più sorgenti dati real-time**

## 🔄 Funzione di Creazione Correlata

Mentre `combineLatestAll` è usato principalmente per appiattire Higher-order Observable,
usa la **Funzione di Creazione** `combineLatest` per normali combinazioni multi-Observable.

```ts
import { combineLatest, interval } from 'rxjs';

// Versione Funzione di Creazione (utilizzo più comune)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Vedi [Capitolo 3: Funzioni di Creazione - combineLatest](/it/guide/creation-functions/combination/combineLatest).

## 🔄 Operatori Correlati

| Operatore | Descrizione |
|---|---|
| [mergeAll](/it/guide/operators/combination/mergeAll) | Sottoscrive tutti gli Observable interni in parallelo |
| [concatAll](/it/guide/operators/combination/concatAll) | Sottoscrive gli Observable interni in ordine |
| [switchAll](/it/guide/operators/combination/switchAll) | Passa al nuovo Observable interno |
| [zipAll](/it/guide/operators/combination/zipAll) | Accoppia valori in ordine corrispondente da ogni Observable interno |

## ⚠️ Note Importanti

### Il Higher-order Observable Deve Completare

`combineLatestAll` attende di raccogliere gli Observable interni fino a quando il Higher-order Observable (Observable esterno) **completa**.

#### ❌ Nessun output perché il Higher-order Observable non completa
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Nessun output
```

#### ✅ Completa con take
```ts
interval(1000).pipe(
  take(3), // Completa dopo 3
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Tutti gli Observable Interni Devono Emettere Almeno Una Volta

Non verranno emessi valori fino a quando tutti gli Observable interni non hanno **emesso almeno una volta**.

```ts
import { of, NEVER } from 'rxjs';
import { combineLatestAll } from 'rxjs';

// Nessun output se anche un solo Observable interno non emette mai
of(
  of(1, 2, 3),
  NEVER // Non emette mai
).pipe(
  combineLatestAll()
).subscribe(console.log); // Nessun output
```

### Utilizzo della Memoria

Nota l'utilizzo della memoria se ci sono molti Observable interni, poiché **gli ultimi valori di tutti gli Observable interni vengono mantenuti in memoria**.
