---
description: reduce è un operatore di conversione RxJS che accumula tutti i valori in uno stream ed emette solo il risultato finale al completamento. È ideale per situazioni dove serve solo il risultato finale dell'aggregazione, come calcolare somme, medie, massimi, minimi, aggregare oggetti e costruire array. A differenza di scan, non emette risultati intermedi e non può essere usato con stream infiniti perché richiede il completamento dello stream.
titleTemplate: ':title | RxJS'
---

# reduce - Emette Solo il Risultato Finale Accumulato

L'operatore `reduce` applica una funzione cumulativa a ogni valore nello stream ed emette **solo il risultato cumulativo finale** al completamento dello stream.
Funziona come `Array.prototype.reduce` per gli array, senza emissione di risultati intermedi.

## 🔰 Sintassi e Utilizzo Base

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Output: 15 (solo il risultato finale)
```

- `acc` è il valore cumulativo, `curr` è il valore corrente.
- I valori vengono accumulati sequenzialmente, partendo dal valore iniziale (`0` in questo caso).
- Nessun valore viene emesso fino al completamento dello stream, e **solo il risultato finale** viene emesso al completamento.

[🌐 Documentazione Ufficiale RxJS - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 Pattern di Utilizzo Tipici

- Calcolare somme, medie, massimi e minimi di numeri
- Aggregare e trasformare oggetti
- Costruire o combinare array
- Quando serve solo il risultato aggregato finale

## 🔍 Differenza da scan

| Operatore | Timing Output | Contenuto Output | Utilizzo |
|:---|:---|:---|:---|
| `reduce` | **Solo una volta al completamento** | Risultato cumulativo finale | Aggregazione dove serve solo il risultato finale |
| `scan` | **Ogni volta per ogni valore** | Tutto inclusi i risultati intermedi | Aggregazione real-time/gestione stato |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 1, 3, 6, 10, 15
```

## 🧠 Esempio di Codice Pratico (con UI)

Questo esempio somma i valori di più campi di input e visualizza il risultato finale al click di un bottone.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Crea campi di input
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Valore ${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// Bottone calcola
const button = document.createElement('button');
button.textContent = 'Calcola Somma';
document.body.appendChild(button);

// Area visualizzazione risultato
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Calcola somma al click del bottone
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Ottieni tutti i valori di input
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Totale: ${total}`;
  console.log('Totale:', total);
});
```

- Al click del bottone, tutti i valori di input vengono sommati e solo il totale finale viene visualizzato.
- I risultati intermedi non vengono emessi.

## 🎯 Esempio di Aggregazione Oggetti

Questo è un esempio pratico di aggregazione di più valori in un oggetto.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: 'Alimentari', price: 500 },
  { category: 'Bevande', price: 200 },
  { category: 'Alimentari', price: 800 },
  { category: 'Bevande', price: 150 },
  { category: 'Alimentari', price: 300 },
];

// Aggrega prezzo totale per categoria
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('Totale per categoria:', result);
});

// Output:
// Totale per categoria: { Alimentari: 1600, Bevande: 350 }
```

## 🎯 Esempio di Costruzione Array

Ecco un esempio di combinazione dei valori dello stream in un array.

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('Array raccolto:', array);
});

// Output:
// Array raccolto: [0, 1, 2, 3, 4]
```

::: tip
Quando costruisci un array, considera l'utilizzo dell'operatore più conciso [`toArray`](/it/guide/operators/utility/toArray).
```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// Output: [0, 1, 2, 3, 4]
```
:::

## 💡 Utilizzo di reduce Type-Safe

Ecco un esempio di utilizzo dell'inferenza dei tipi di TypeScript.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// Aggrega conteggio per tipo di azione
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('Aggregazione azioni:', result);
});

// Output:
// Aggregazione azioni: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ Note

### ❌ Gli Stream Infiniti Non Completano (Importante)

> [!WARNING]
> **`reduce` non emetterà un singolo valore fino a quando non viene chiamato `complete()`.** Gli stream infiniti (`interval`, `fromEvent`, ecc.) causano problemi in pratica, poiché nessun valore è mai disponibile.

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Esempio sbagliato: Stream infinito quindi nessun valore viene emesso
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Nessun output (lo stream non completa)
```

**Contromisura 1: Usa `scan` quando serve aggregazione continua**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Esempio corretto: Ottieni risultati intermedi in tempo reale
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 0, 1, 3, 6, 10 (emette valore cumulativo ogni volta)
```

**Contromisura 2: Se serve solo il valore finale, usa `scan` + `takeLast(1)`**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ Esempio corretto: Accumula con scan, ottieni solo valore finale
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// Output: 10 (solo risultato finale)
```

**Contromisura 3: Usa `take` per specificare la condizione di fine**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ Esempio corretto: Imposta condizione di fine con take
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 10
```

> [!TIP]
> **Criteri di Selezione**:
> - Servono risultati intermedi → `scan`
> - Serve solo il risultato finale & completamento stream garantito → `reduce`
> - Serve solo risultato finale & stream infinito → `scan` + `takeLast(1)` oppure `take` + `reduce`

### Utilizzo Memoria

Quando il valore cumulativo è un oggetto o array grande, bisogna considerare l'utilizzo di memoria.

```ts
// Esempio che richiede attenzione alla memoria
from(largeDataArray).pipe(
  reduce((acc, item) => {
    acc.push(item); // Accumula grandi quantità di dati
    return acc;
  }, [])
).subscribe();
```

## 📚 Operatori Correlati

- [`scan`](/it/guide/operators/transformation/scan) - Emette un risultato intermedio per ogni valore
- [`toArray`](/it/guide/operators/utility/toArray) - Combina tutti i valori in un array
- [`count`](https://rxjs.dev/api/operators/count) - Conta il numero di valori
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - Ottieni valori minimo e massimo

## Riepilogo

L'operatore `reduce` accumula tutti i valori in uno stream ed emette **solo il risultato finale al completamento**. È adatto quando non servono risultati intermedi e serve solo il risultato aggregato finale. Tuttavia, poiché nessun risultato viene ottenuto se lo stream non completa, devi usare `scan` per stream infiniti, o impostare una condizione di uscita con `take` o simili.
