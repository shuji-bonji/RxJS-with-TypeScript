---
description: "takeWhile è un operatore di filtraggio RxJS che continua a prendere valori mentre la condizione specificata è soddisfatta e completa lo stream quando la condizione diventa falsa. È ideale per situazioni dove vuoi controllare uno stream con condizioni dinamiche, come acquisizione dati fino a una soglia, elaborazione basata su priorità, paginazione, ecc. L'opzione inclusive permette di includere i valori per cui la condizione diventa falsa."
---

# takeWhile - Prendi Valori Mentre la Condizione è Soddisfatta

L'operatore `takeWhile` continua a prendere valori **mentre la condizione specificata è soddisfatta**, e completa lo stream quando la condizione diventa `false`.


## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Completo')
});
// Output: 0, 1, 2, 3, 4, Completo
```

**Flusso di operazione**:
1. 0 viene emesso → `0 < 5` è `true` → Output
2. 1 viene emesso → `1 < 5` è `true` → Output
3. 2 viene emesso → `2 < 5` è `true` → Output
4. 3 viene emesso → `3 < 5` è `true` → Output
5. 4 viene emesso → `4 < 5` è `true` → Output
6. 5 viene emesso → `5 < 5` è `false` → Completo (5 non viene emesso)

[🌐 Documentazione Ufficiale RxJS - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)


## 🆚 Confronto con take

`take` e `takeWhile` hanno condizioni di acquisizione diverse.

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs';

const source$ = interval(1000);

// take: Controllo per conteggio
source$.pipe(
  take(5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// takeWhile: Controllo per condizione
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4
```

| Operatore | Metodo di Controllo | Condizione di Completamento | Ultimo Valore |
|---|---|---|---|
| `take(n)` | Conteggio | Dopo n valori | Include l'n-esimo valore |
| `takeWhile(predicate)` | Funzione condizione | Quando la condizione diventa `false` | Non include il valore che è diventato `false`* |

\* Per default, il valore che è diventato `false` non viene emesso, ma può essere incluso con l'opzione `inclusive: true`


## 🎯 Opzione inclusive

Se vuoi includere il valore per cui la condizione è diventata `false`, specifica `inclusive: true`.

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

const numbers$ = range(0, 10);

// Default (inclusive: false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// inclusive: true
numbers$.pipe(
  takeWhile(n => n < 5, true)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5 (include 5 che ha reso la condizione falsa)
```


## 💡 Pattern di Utilizzo Tipici

1. **Acquisizione Dati Fino a Soglia**
   ```ts
   import { interval } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   // Simulazione sensore temperatura
   const temperature$ = interval(100).pipe(
     map(() => 20 + Math.random() * 15)
   );

   // Registra solo mentre sotto i 30 gradi
   temperature$.pipe(
     takeWhile(temp => temp < 30)
   ).subscribe({
     next: temp => console.log(`Temperatura: ${temp.toFixed(1)}°C`),
     complete: () => console.log('Attenzione: Temperatura superiore a 30 gradi!')
   });
   ```

2. **Elaborazione Array Condizionale**
   ```ts
   import { from } from 'rxjs';
   import { takeWhile } from 'rxjs';

   interface Task {
     id: number;
     priority: 'high' | 'medium' | 'low';
     completed: boolean;
   }

   const tasks$ = from([
     { id: 1, priority: 'high' as const, completed: false },
     { id: 2, priority: 'high' as const, completed: false },
     { id: 3, priority: 'medium' as const, completed: false },
     { id: 4, priority: 'low' as const, completed: false },
   ] as Task[]);

   // Elabora solo mentre la priorità è alta
   tasks$.pipe(
     takeWhile(task => task.priority === 'high')
   ).subscribe(task => {
     console.log(`Elaborazione task ${task.id}`);
   });
   // Output: Elaborazione task 1, Elaborazione task 2
   ```

3. **Elaborazione Paginazione**
   ```ts
   import { range } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   interface Page {
     pageNumber: number;
     hasMore: boolean;
   }

   const pages$ = range(1, 10).pipe(
     map(pageNum => ({
       pageNumber: pageNum,
       hasMore: pageNum < 5
     } as Page))
   );

   // Carica pagine solo mentre hasMore è true
   pages$.pipe(
     takeWhile(page => page.hasMore, true) // inclusive: true
   ).subscribe(page => {
     console.log(`Caricamento pagina ${page.pageNumber}`);
   });
   // Output: Caricamento pagina 1~5
   ```


## 🧠 Esempio di Codice Pratico (Limite Contatore)

Esempio di continuare il conteggio fino a raggiungere una condizione specifica.

```ts
import { fromEvent, interval } from 'rxjs';
import { takeWhile, scan, switchMap } from 'rxjs';

// Crea elementi UI
const container = document.createElement('div');
document.body.appendChild(container);

const startButton = document.createElement('button');
startButton.textContent = 'Avvia Conteggio';
container.appendChild(startButton);

const counter = document.createElement('div');
counter.style.fontSize = '24px';
counter.style.marginTop = '10px';
counter.textContent = 'Conteggio: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'Conta mentre è sotto 10';
container.appendChild(message);

// Avvia conteggio al click del bottone
fromEvent(startButton, 'click').pipe(
  switchMap(() =>
    interval(500).pipe(
      scan(count => count + 1, 0),
      takeWhile(count => count < 10)
    )
  )
).subscribe({
  next: (count) => {
    counter.textContent = `Conteggio: ${count}`;
    startButton.disabled = true;
  },
  complete: () => {
    message.textContent = 'Completato dopo aver raggiunto 10!';
    message.style.color = 'green';
    startButton.disabled = false;
  }
});
```

Questo codice conta da 0 a 9 e si completa automaticamente appena prima di raggiungere 10.


## 🎯 Confronto con skipWhile

`takeWhile` e `skipWhile` hanno comportamento opposto.

```ts
import { range } from 'rxjs';
import { takeWhile, skipWhile } from 'rxjs';

const numbers$ = range(0, 10);

// takeWhile: Prendi mentre la condizione è soddisfatta
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// skipWhile: Salta mentre la condizione è soddisfatta
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

| Operatore | Comportamento | Timing Completamento |
|---|---|---|
| `takeWhile(predicate)` | **Prendi** mentre la condizione è soddisfatta | Quando la condizione diventa `false` |
| `skipWhile(predicate)` | **Salta** mentre la condizione è soddisfatta | Quando lo stream sorgente completa |


## 📋 Utilizzo Type-Safe

Esempio di implementazione type-safe utilizzando i generics di TypeScript.

```ts
import { Observable, from } from 'rxjs';
import { takeWhile } from 'rxjs';

interface SensorReading {
  timestamp: Date;
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical';
}

function getReadingsUntilWarning(
  readings$: Observable<SensorReading>
): Observable<SensorReading> {
  return readings$.pipe(
    takeWhile(reading => reading.status === 'normal')
  );
}

// Esempio di utilizzo
const readings$ = from([
  { timestamp: new Date(), value: 25, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 28, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 32, unit: '°C', status: 'warning' as const },
  { timestamp: new Date(), value: 35, unit: '°C', status: 'critical' as const },
] as SensorReading[]);

getReadingsUntilWarning(readings$).subscribe(reading => {
  console.log(`${reading.value}${reading.unit} - ${reading.status}`);
});
// Output:
// 25°C - normal
// 28°C - normal
```


## 🔄 Differenza Tra takeWhile e filter

`takeWhile` differisce da `filter` in quanto completa.

```ts
import { range } from 'rxjs';
import { takeWhile, filter } from 'rxjs';

const numbers$ = range(0, 10);

// filter: Passa solo i valori che soddisfano la condizione (lo stream continua)
numbers$.pipe(
  filter(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter completo')
});
// Output: 0, 1, 2, 3, 4, filter completo

// takeWhile: Solo mentre la condizione è soddisfatta (completa quando falsa)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('takeWhile completo')
});
// Output: 0, 1, 2, 3, 4, takeWhile completo
```

| Operatore | Comportamento | Completamento Stream |
|---|---|---|
| `filter(predicate)` | Passa solo i valori che soddisfano la condizione | Quando lo stream sorgente completa |
| `takeWhile(predicate)` | Prendi mentre la condizione è soddisfatta | Quando la condizione diventa `false` |


## ⚠️ Errori Comuni

> [!NOTE]
> `takeWhile` completa senza emettere nulla se la condizione è `false` dall'inizio. Assicurati che la condizione sia impostata appropriatamente.

### Sbagliato: La Condizione è Falsa Dall'Inizio

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ❌ Esempio sbagliato: La condizione è falsa al primo valore
range(5, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Nessun output (la condizione è falsa al primo valore 5)
```

### Corretto: Verifica la Condizione

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ✅ Esempio corretto: Imposta la condizione appropriatamente
range(0, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4
```


## 🎓 Riepilogo

### Quando Usare takeWhile
- ✅ Quando vuoi controllare lo stream con condizioni dinamiche
- ✅ Quando vuoi acquisire dati fino a una soglia
- ✅ Quando vuoi elaborare solo mentre uno stato specifico continua
- ✅ Quando serve completamento anticipato basato su condizione

### Quando Usare take
- ✅ Quando il numero da acquisire è fisso
- ✅ Quando serve un semplice limite di conteggio

### Quando Usare filter
- ✅ Quando vuoi estrarre solo i valori che soddisfano una condizione dall'intero stream
- ✅ Quando non vuoi completare lo stream

### Note
- ⚠️ Se la condizione è `false` dall'inizio, completa senza emettere nulla
- ⚠️ Per default, i valori per cui la condizione diventa `false` non vengono emessi (possono essere inclusi con `inclusive: true`)
- ⚠️ Con stream infiniti dove la condizione è sempre `true`, continua per sempre


## 🚀 Prossimi Passi

- **[take](/it/guide/operators/filtering/take)** - Impara come prendere i primi N valori
- **[takeLast](/it/guide/operators/filtering/takeLast)** - Impara come prendere gli ultimi N valori
- **[takeUntil](../utility/takeUntil)** - Impara come prendere valori fino a quando un altro Observable si attiva
- **[filter](/it/guide/operators/filtering/filter)** - Impara come filtrare in base a condizioni
- **[Esempi Pratici Operatori di Filtraggio](/it/guide/operators/filtering/practical-use-cases)** - Impara casi d'uso reali
