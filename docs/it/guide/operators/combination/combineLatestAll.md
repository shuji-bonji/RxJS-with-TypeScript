---
description: "combineLatestAll è un operatore che prende un Observable di ordine superiore (Observable of Observable) e combina l'ultimo valore di ciascuno di essi una volta che tutti gli Observable interni sono stati attivati almeno una volta."
---

# combineLatestAll - combina i valori più recenti dell'Observable interno

L'operatore combineLatestAll prende un **Osservabile di ordine superiore** (Observable of Observables),
**Una volta che tutti gli Observable interni si sono attivati almeno una volta**, i **valori più recenti** di ciascuno di essi vengono combinati ed emessi come **array**.

## 🔰 Sintassi e uso di base

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3Due interniObservableconHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Tutti gli interniObservablehanno almeno1spara una volta, combina i valori più recenti
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Uscita:
// [1, 3, 0] ← Quando tutti hanno sparato almeno1Quando tutti hanno sparato almeno una volta (dopo2secondi dopo)
// [2, 3, 0] ← 1Quando il secondoObservableviene sparato (dopo 2 secondi)2spara (dopo3secondi dopo)
// [2, 3, 1] ← 3Quando il secondoObservableviene sparato (dopo 2 secondi)1spara (dopo4secondi dopo)
```

- Raccogliere gli Observable interni quando l'Observable di ordine superiore è **completo**.
- Iniziare a combinare quando tutti gli Observable interni si sono attivati** almeno una volta.
- Ogni volta che un Observable interno emette un valore, **combinare** tutti gli ultimi valori e **output**.

[🌐 Documentazione ufficiale di RxJS - combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Tipico modello di utilizzo.

- **Combinare i risultati più recenti di più chiamate API**
- Sincronizzare i valori più recenti di più input di moduli**
- **Integrare più fonti di dati in tempo reale**

## 🧠 Esempi pratici di codice

Esempio di combinazione degli ultimi risultati di più chiamate API

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3Due simulazioniAPIChiamata di creazione (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Informazioni sull'utente (1Completato in secondi,3aggiornate una volta)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Utente: User${n}`)
  ),
  // API 2: Numero di notifiche (0.5Completato in secondi,4aggiornate una volta)
  timer(0, 500).pipe(
    take(4),
    map(n => `Notifiche: ${n}Numero di notifiche`)
  ),
  // API 3: Stato2Completato in secondi,2aggiornate una volta)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Stato: Non in linea' : 'Stato: In linea')
  )
);

// TuttiAPIValore ultimo combinato delle chiamate
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Ultimo stato:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- Tre chiamate API vengono eseguite in parallelo.
- Dopo che tutte sono state eseguite almeno una volta**, vengono visualizzati i risultati combinati.
- Ogni volta che una API viene aggiornata, viene visualizzata la **combinazione più recente**.

## 🔄 Creation Function correlata

CombineLatestAll è usato principalmente per appiattire gli Observable di ordine superiore,
Utilizzare la **funzione di creazione** `combineLatest` per le normali combinazioni di Observable multipli.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation FunctionEdizione (uso più generale)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Vedere [Capitolo 3 Creation Function - combineLatest](/it/guide/creation-functions/combination/combineLatest).

## 🔄 Operatori correlati.

| Operatore. | Descrizione. |
|---|---|
| [mergeAll](./mergeAll) | Sottoscrive tutti gli Observable interni in parallelo. |
| [concatAll](./concatAll) | Sottoscrivere gli Observable interni in sequenza. |
| [switchAll](./switchAll) | Passa a un nuovo Observable interno. |
| [zipAll](./zipAll) | Accoppia i valori di ogni Observable interno nell'ordine corrispondente |

## ⚠️ Note.

### L'Observable di ordine superiore deve essere completato.

combineLatestAll attende la raccolta degli Observable interni **fino a quando** l'Observable di ordine superiore (Observable esterno) è **completato**.

#### ❌ L'Observable di ordine superiore non viene completato, quindi non viene emesso nulla.

```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Non viene emesso nulla
```

#### ✅ Take per completare

```ts
interval(1000).pipe(
  take(3), // 3Completato in una sessione
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Tutti gli Observable interni devono sparare almeno una volta

Nessun valore viene emesso finché tutti gli Observable interni non si sono attivati **almeno una volta**.

```ts
// 1Se uno qualsiasi dei parametri interniObservableNon viene emesso nulla se ci sono
of(
  of(1, 2, 3),
  NEVER // Non viene mai attivato per sempre.
).pipe(
  combineLatestAll()
).subscribe(console.log); // Non viene emesso nulla
```

### Utilizzo della memoria.

Si noti l'utilizzo della memoria se ci sono molti Observable interni, poiché i **valori più recenti di tutti gli Observable interni sono tenuti in memoria**.
