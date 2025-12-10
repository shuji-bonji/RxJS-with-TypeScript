---
description: "L'operatore skipLast salta gli ultimi N valori negli stream Observable ed emette solo i valori precedenti: Perfetto per escludere dati pending non confermati"
---

# skipLast - Salta gli Ultimi N Valori

L'operatore `skipLast` **salta gli ultimi N valori** emessi dall'Observable sorgente ed emette solo i valori precedenti. Mantiene gli ultimi N valori in un buffer fino al completamento dello stream ed emette il resto.

## 🔰 Sintassi e Utilizzo Base

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // Da 0 a 9

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 vengono saltati)
```

**Flusso di operazione**:
1. Lo stream emette 0, 1, 2, ...
2. Mantiene gli ultimi 3 valori (7, 8, 9) nel buffer
3. Emette i valori che eccedono la dimensione del buffer (0~6)
4. Quando lo stream completa, i valori del buffer (7, 8, 9) vengono scartati senza emissione

[🌐 Documentazione Ufficiale RxJS - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Pattern di Utilizzo Tipici

- **Escludi ultimi dati**: Escludi ultimi dati non confermati
- **Elaborazione batch**: Escludi dati pending prima che l'elaborazione completi
- **Validazione dati**: Quando serve validazione sui valori successivi
- **Elaborazione dati finalizzati ritardati**: Quando gli ultimi N elementi non sono finalizzati

## 🧠 Esempio di Codice Pratico 1: Pipeline Elaborazione Dati

Esempio di salto degli ultimi dati pending nell'elaborazione dati.

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// Crea UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Pipeline Elaborazione Dati';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = 'Salta ultimi 2 elementi (dati pending) prima dell\'elaborazione';
container.appendChild(description);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

interface DataPoint {
  id: number;
  value: number;
  status: 'processing' | 'confirmed' | 'skipped';
}

// Stream dati (10 elementi)
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// Emetti dati ogni 0.5 secondi
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // Salta ultimi 2 elementi
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ Confermato</strong>
      ID: ${item.id} |
      Valore: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // Mostra elementi saltati
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ Saltato</strong>
        ID: ${item.id} |
        Valore: ${item.value} |
        (Dati pending)
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `Elaborazione completata: ${data.length - 2} confermati, 2 saltati`;
    output.appendChild(summary);
  }
});
```

- I dati vengono elaborati sequenzialmente, ma gli ultimi 2 elementi sono trattati come pending e saltati.
- Dopo il completamento, vengono mostrati anche gli elementi saltati.

## 🎯 Esempio di Codice Pratico 2: Filtraggio Log

Esempio di salto degli ultimi log non confermati da uno stream di log.

```ts
import { interval } from 'rxjs';
import { skipLast, map, take } from 'rxjs';

// Crea UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Monitoraggio Log';
container.appendChild(title);

const info = document.createElement('div');
info.style.marginBottom = '10px';
info.textContent = 'Gli ultimi 3 log vengono saltati come pending di conferma';
info.style.color = '#666';
container.appendChild(info);

const confirmedLogs = document.createElement('div');
confirmedLogs.innerHTML = '<strong>📋 Log Confermati:</strong>';
confirmedLogs.style.marginBottom = '10px';
container.appendChild(confirmedLogs);

const confirmedList = document.createElement('div');
confirmedList.style.border = '1px solid #4CAF50';
confirmedList.style.padding = '10px';
confirmedList.style.backgroundColor = '#f1f8e9';
confirmedList.style.minHeight = '100px';
container.appendChild(confirmedList);

const pendingLogs = document.createElement('div');
pendingLogs.innerHTML = '<strong>⏳ Log Pending (Saltati):</strong>';
pendingLogs.style.marginTop = '10px';
pendingLogs.style.marginBottom = '10px';
container.appendChild(pendingLogs);

const pendingList = document.createElement('div');
pendingList.style.border = '1px solid #FF9800';
pendingList.style.padding = '10px';
pendingList.style.backgroundColor = '#fff3e0';
pendingList.style.minHeight = '60px';
container.appendChild(pendingList);

interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error';
  message: string;
}

// Genera log (12 totali, ogni 1 secondo)
const logs$ = interval(1000).pipe(
  take(12),
  map(i => {
    const levels: ('info' | 'warn' | 'error')[] = ['info', 'warn', 'error'];
    const messages = [
      'Login utente',
      'Fetch dati avviato',
      'Cache aggiornata',
      'Errore connessione',
      'Retry eseguito',
      'Elaborazione dati completata'
    ];
    return {
      id: i,
      timestamp: new Date(),
      level: levels[Math.floor(Math.random() * levels.length)],
      message: messages[Math.floor(Math.random() * messages.length)]
    } as LogEntry;
  })
);

const allLogs: LogEntry[] = [];

// Registra tutti i log (per verifica)
logs$.subscribe(log => {
  allLogs.push(log);
});

// Mostra log confermati, saltando ultimi 3
logs$.pipe(
  skipLast(3)
).subscribe({
  next: log => {
    const logDiv = document.createElement('div');
    logDiv.style.padding = '3px';
    logDiv.style.marginBottom = '3px';
    const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
    logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
    confirmedList.appendChild(logDiv);
  },
  complete: () => {
    // Mostra ultimi 3 log (log saltati)
    const skippedLogs = allLogs.slice(-3);
    skippedLogs.forEach(log => {
      const logDiv = document.createElement('div');
      logDiv.style.padding = '3px';
      logDiv.style.marginBottom = '3px';
      const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
      logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
      pendingList.appendChild(logDiv);
    });
  }
});
```

- I log vengono aggiunti sequenzialmente, ma gli ultimi 3 vengono saltati come pending di conferma.
- Dopo il completamento, vengono mostrati anche i log saltati.

## 🆚 Confronto con Operatori Simili

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // Da 0 a 9

// skipLast: Salta ultimi N valori
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6

// takeLast: Prendi solo ultimi N valori
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9

// skip: Salta primi N valori
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, 8, 9
```

| Operatore | Posizione Skip | Timing Output | Richiede Completamento |
|:---|:---|:---|:---|
| `skipLast(n)` | Ultimi n valori | Output quando buffer è superato | Richiesto |
| `takeLast(n)` | Tutti tranne ultimi n | Output tutti insieme dopo completamento | Richiesto |
| `skip(n)` | Primi n valori | Output immediato | Non richiesto |

**Differenze visive**:

```
Input: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 saltati]
                                   ^Ultimi 3

takeLast(3): [0~6 saltati] | 7, 8, 9
                             ^Solo ultimi 3

skip(3): [0, 1, 2 saltati] | 3, 4, 5, 6, 7, 8, 9
          ^Primi 3
```

## ⚠️ Note Importanti

### 1. Comportamento con Stream Infiniti

Poiché `skipLast` non può identificare gli ultimi N valori fino al completamento, non funziona come previsto con stream infiniti.

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Esempio sbagliato: Usa skipLast con stream infinito
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0 (dopo 3 secondi), 1 (dopo 4 secondi), 2 (dopo 5 secondi), ...
// Tutti i valori continuano ad essere emessi all'infinito con ritardo N
// Gli ultimi 3 rimangono nel buffer per sempre e non vengono mai emessi
```

Con stream infiniti, poiché gli ultimi N valori non sono determinati, tutti i valori continuano ad essere emessi con un ritardo di N. Poiché non ci sono veri "ultimi N", lo scopo originale di `skipLast` non può essere raggiunto.

**Soluzione**: Rendilo uno stream finito con `take`

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Esempio corretto: Rendi stream finito prima di usare skipLast
interval(1000).pipe(
  take(10),      // Completa con primi 10 valori
  skipLast(3)    // Salta ultimi 3
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 vengono saltati)
```

### 2. Attenzione alla Dimensione del Buffer

`skipLast(n)` mantiene sempre n valori nel buffer.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ Attenzione: Mantiene 1000 elementi nel buffer
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. Ritardo Output

`skipLast(n)` non emette nulla finché n buffer non sono riempiti.

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('Input:', val)),
  skipLast(2)
).subscribe(val => console.log('Output:', val));
// Input: 0
// Input: 1
// Input: 2
// Output: 0  ← Output inizia dopo che buffer si riempie con 2
// Input: 3
// Output: 1
// Input: 4
// Output: 2
// Completo (3, 4 vengono saltati)
```

### 4. Comportamento skipLast(0)

`skipLast(0)` non salta nulla.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

range(0, 5).pipe(
  skipLast(0)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4 (tutti emessi)
```

## 💡 Pattern di Combinazione Pratici

### Pattern 1: Ottieni Solo la Sezione Centrale

Salta sia l'inizio che la fine per ottenere solo la sezione centrale

```ts
import { range } from 'rxjs';
import { skip, skipLast } from 'rxjs';

range(0, 10).pipe(
  skip(2),      // Salta primi 2
  skipLast(2)   // Salta ultimi 2
).subscribe(console.log);
// Output: 2, 3, 4, 5, 6, 7
```

### Pattern 2: Validazione Dati

Quando serve validazione sui valori successivi

```ts
import { from } from 'rxjs';
import { skipLast, map } from 'rxjs';

interface Transaction {
  id: number;
  amount: number;
  pending: boolean;
}

const transactions$ = from([
  { id: 1, amount: 100, pending: false },
  { id: 2, amount: 200, pending: false },
  { id: 3, amount: 150, pending: false },
  { id: 4, amount: 300, pending: true },  // Pending
  { id: 5, amount: 250, pending: true }   // Pending
]);

// Salta transazioni pending (ultime 2)
transactions$.pipe(
  skipLast(2)
).subscribe(tx => {
  console.log(`Confermata: ID ${tx.id}, Importo ${tx.amount}`);
});
// Output:
// Confermata: ID 1, Importo 100
// Confermata: ID 2, Importo 200
// Confermata: ID 3, Importo 150
```

### Pattern 3: Elaborazione Window

Elaborazione window con dati escludendo gli ultimi N elementi

```ts
import { range } from 'rxjs';
import { skipLast, bufferCount } from 'rxjs';

range(0, 10).pipe(
  skipLast(2),      // Salta ultimi 2
  bufferCount(3, 1) // Window di 3
).subscribe(window => {
  console.log('Window:', window);
});
// Output:
// Window: [0, 1, 2]
// Window: [1, 2, 3]
// Window: [2, 3, 4]
// ...
```

## 📚 Operatori Correlati

- **[skip](/it/guide/operators/filtering/skip)** - Salta primi N valori
- **[takeLast](/it/guide/operators/filtering/takeLast)** - Prendi solo ultimi N valori
- **[take](/it/guide/operators/filtering/take)** - Prendi solo primi N valori
- **[skipUntil](/it/guide/operators/filtering/skipUntil)** - Salta fino a quando un altro Observable si attiva
- **[skipWhile](/it/guide/operators/filtering/skipWhile)** - Salta mentre la condizione è soddisfatta

## Riepilogo

L'operatore `skipLast` salta gli ultimi N valori nello stream.

- ✅ Ideale quando gli ultimi N dati non servono
- ✅ Utile per escludere dati non confermati
- ✅ La dimensione del buffer è solo N (efficiente in memoria)
- ✅ Richiede completamento stream
- ⚠️ Non può essere usato con stream infiniti
- ⚠️ Nessun output finché il buffer non si riempie con N valori
- ⚠️ Spesso deve essere combinato con `take` per rendere lo stream finito
