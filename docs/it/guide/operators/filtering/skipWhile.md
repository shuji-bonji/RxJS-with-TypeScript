---
description: L'operatore skipWhile salta i valori mentre la condizione specificata è soddisfatta ed emette tutti i valori successivi dal punto in cui la condizione diventa falsa. È utile quando vuoi controllare uno stream con una condizione di avvio dinamica.
---

# skipWhile - Salta Valori Mentre la Condizione è Soddisfatta

L'operatore `skipWhile` continua a saltare i valori **mentre la condizione specificata è soddisfatta**, ed emette **tutti i valori successivi** dal punto in cui la condizione diventa `false`.

## 🔰 Sintassi e Utilizzo Base

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // Da 0 a 9

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

**Flusso di operazione**:
1. 0 viene emesso → `0 < 5` è `true` → Salta
2. 1 viene emesso → `1 < 5` è `true` → Salta
3. 2 viene emesso → `2 < 5` è `true` → Salta
4. 3 viene emesso → `3 < 5` è `true` → Salta
5. 4 viene emesso → `4 < 5` è `true` → Salta
6. 5 viene emesso → `5 < 5` è `false` → Inizia output
7. 6 e successivi → Tutti emessi (condizione non viene rivalutata)

[🌐 Documentazione Ufficiale RxJS - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 Pattern di Utilizzo Tipici

- **Salta dati iniziali non necessari**: Escludi dati durante periodo di riscaldamento
- **Salta fino al raggiungimento della soglia**: Attendi fino a quando condizioni specifiche sono soddisfatte
- **Salta righe header**: Escludi header CSV, ecc.
- **Salta periodo preparazione**: Attendi fino a quando il sistema è pronto

## 🧠 Esempio di Codice Pratico 1: Salta Periodo Riscaldamento Sensore

Esempio di salto dei dati iniziali fino a quando il sensore si stabilizza.

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

// Crea UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Monitoraggio Sensore Temperatura';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginBottom = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#fff3e0';
status.style.border = '1px solid #FF9800';
status.textContent = '🔄 Sensore in preparazione... (Misurazione inizia quando temperatura >= 20°C)';
container.appendChild(status);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

let isWarmedUp = false;

// Simulazione sensore temperatura (si riscalda gradualmente)
interval(500).pipe(
  take(20),
  map(i => {
    // Inizia temperatura bassa, aumenta gradualmente
    const baseTemp = 15 + i * 0.5;
    const noise = (Math.random() - 0.5) * 2;
    return baseTemp + noise;
  }),
  skipWhile(temp => temp < 20) // Salta sotto 20°C
).subscribe({
  next: temp => {
    // Aggiorna stato quando arriva primo valore
    if (!isWarmedUp) {
      isWarmedUp = true;
      status.textContent = '✅ Sensore pronto (Misurazione iniziata)';
      status.style.backgroundColor = '#e8f5e9';
      status.style.borderColor = '#4CAF50';
    }

    const log = document.createElement('div');
    log.style.padding = '5px';
    log.style.marginBottom = '3px';
    log.style.backgroundColor = temp > 25 ? '#ffebee' : '#f1f8e9';
    log.textContent = `[${new Date().toLocaleTimeString()}] Temperatura: ${temp.toFixed(1)}°C`;
    output.insertBefore(log, output.firstChild);

    // Mostra massimo 10 elementi
    while (output.children.length > 10) {
      output.removeChild(output.lastChild!);
    }
  },
  complete: () => {
    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = 'Misurazione completata';
    container.appendChild(summary);
  }
});
```

- I dati vengono saltati mentre il sensore è sotto 20°C.
- Tutti i dati vengono registrati dal punto in cui raggiunge 20°C o più.

## 🎯 Esempio di Codice Pratico 2: Elaborazione Eventi Dopo Ready

Esempio di salto degli eventi fino al completamento dell'inizializzazione del sistema.

```ts
import { fromEvent, merge, Subject } from 'rxjs';
import { skipWhile, map, tap } from 'rxjs';

// Crea UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Sistema Elaborazione Eventi';
container.appendChild(title);

const initButton = document.createElement('button');
initButton.textContent = 'Completa Inizializzazione';
initButton.style.marginRight = '10px';
container.appendChild(initButton);

const eventButton = document.createElement('button');
eventButton.textContent = 'Attiva Evento';
container.appendChild(eventButton);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
statusDiv.style.padding = '10px';
statusDiv.style.backgroundColor = '#ffebee';
statusDiv.style.border = '1px solid #f44336';
statusDiv.innerHTML = '<strong>⏸️ Sistema Non Inizializzato</strong><br>Gli eventi verranno saltati';
container.appendChild(statusDiv);

const eventLog = document.createElement('div');
eventLog.style.marginTop = '10px';
eventLog.style.border = '1px solid #ccc';
eventLog.style.padding = '10px';
eventLog.style.minHeight = '100px';
container.appendChild(eventLog);

// Stato inizializzazione
let isInitialized = false;
const initSubject = new Subject<boolean>();

// Pulsante inizializzazione
fromEvent(initButton, 'click').subscribe(() => {
  if (!isInitialized) {
    isInitialized = true;
    initSubject.next(true);
    statusDiv.style.backgroundColor = '#e8f5e9';
    statusDiv.style.borderColor = '#4CAF50';
    statusDiv.innerHTML = '<strong>✅ Sistema Inizializzato</strong><br>Elaborazione eventi';
    initButton.disabled = true;
  }
});

// Elaborazione eventi (salta fino a inizializzazione)
let eventCount = 0;
fromEvent(eventButton, 'click').pipe(
  map(() => {
    eventCount++;
    return {
      id: eventCount,
      timestamp: new Date(),
      initialized: isInitialized
    };
  }),
  tap(event => {
    if (!event.initialized) {
      const skipLog = document.createElement('div');
      skipLog.style.padding = '5px';
      skipLog.style.marginBottom = '3px';
      skipLog.style.color = '#999';
      skipLog.textContent = `⏭️ Evento #${event.id} saltato (non inizializzato)`;
      eventLog.insertBefore(skipLog, eventLog.firstChild);
    }
  }),
  skipWhile(event => !event.initialized)
).subscribe(event => {
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.marginBottom = '3px';
  log.style.backgroundColor = '#e8f5e9';
  log.style.border = '1px solid #4CAF50';
  log.innerHTML = `
    <strong>✅ Evento #${event.id} Elaborato</strong>
    [${event.timestamp.toLocaleTimeString()}]
  `;
  eventLog.insertBefore(log, eventLog.firstChild);

  // Mostra massimo 10 elementi
  while (eventLog.children.length > 10) {
    eventLog.removeChild(eventLog.lastChild!);
  }
});
```

- Tutti gli eventi vengono saltati fino a quando il sistema è inizializzato.
- Dopo il completamento dell'inizializzazione, tutti gli eventi vengono elaborati.

## 🆚 Confronto con Operatori Simili

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs';

const numbers$ = range(0, 10); // Da 0 a 9

// skipWhile: Salta mentre la condizione è soddisfatta, poi emetti tutti
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9

// takeWhile: Prendi solo mentre la condizione è soddisfatta
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// skip: Salta primi N valori
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9

// filter: Passa solo valori che corrispondono alla condizione (valutato per tutti)
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

| Operatore | Comportamento | Rivaluta Condizione | Timing Completamento |
|:---|:---|:---|:---|
| `skipWhile(predicate)` | Salta mentre condizione è soddisfatta | No (termina una volta falsa) | Quando stream sorgente completa |
| `takeWhile(predicate)` | Prendi mentre condizione è soddisfatta | Ogni volta | Quando condizione diventa falsa |
| `skip(n)` | Salta primi n valori | Nessuna (basato su conteggio) | Quando stream sorgente completa |
| `filter(predicate)` | Passa solo valori corrispondenti | **Ogni volta** | Quando stream sorgente completa |

**Differenze visive**:

```
Input: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 saltati] | 5, 4, 3, 2, 1, 0
                      ^Tutto emesso dopo che condizione diventa falsa

filter(n => n >= 5):
[0,1,2,3,4 esclusi] 5 [4,3,2,1,0 esclusi]
                     ^Solo emetti valori corrispondenti (valutato ogni volta)

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [ignora tutto dopo 5 e completa]
```

## ⚠️ Note Importanti

### 1. La Condizione Non Viene Rivalutata Una Volta Falsa

Questa è la differenza più grande da `filter`.

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: Una volta che condizione diventa falsa, emetti tutti i valori successivi
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// Output: skipWhile: 5, 4, 3, 2, 1 (tutti dopo 5)

// filter: Valuta condizione ogni volta
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// Output: filter: 5 (solo 5)
```

### 2. Se la Condizione è Falsa Dall'Inizio

Se la condizione è `false` dall'inizio, tutti i valori vengono emessi.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(5, 5).pipe( // Da 5 a 9
  skipWhile(n => n < 3) // Condizione è falsa dall'inizio
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9 (tutti emessi)
```

### 3. Se Tutti i Valori Soddisfano la Condizione

Se tutti i valori soddisfano la condizione, non viene emesso nulla.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(0, 5).pipe( // Da 0 a 4
  skipWhile(n => n < 10) // Tutti i valori soddisfano la condizione
).subscribe({
  next: console.log,
  complete: () => console.log('Completo (niente emesso)')
});
// Output: Completo (niente emesso)
```

### 4. Tipi TypeScript

`skipWhile` non cambia il tipo.

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// Il tipo rimane Observable<User>
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// Output: Charlie (ID: 3), Dave (ID: 4)
```

## 💡 Pattern di Combinazione Pratici

### Pattern 1: Salta Riga Header

Salta righe header CSV

```ts
import { from } from 'rxjs';
import { skipWhile, map } from 'rxjs';

const csvLines$ = from([
  'Nome,Età,Città',     // Riga header
  'Alice,25,Tokyo',
  'Bob,30,Osaka',
  'Charlie,35,Kyoto'
]);

let isFirstLine = true;

csvLines$.pipe(
  skipWhile(() => {
    if (isFirstLine) {
      isFirstLine = false;
      return true; // Salta prima riga (header)
    }
    return false;
  }),
  map(line => {
    const [name, age, city] = line.split(',');
    return { name, age: Number(age), city };
  })
).subscribe(console.log);
// Output:
// { name: 'Alice', age: 25, city: 'Tokyo' }
// { name: 'Bob', age: 30, city: 'Osaka' }
// { name: 'Charlie', age: 35, city: 'Kyoto' }
```

### Pattern 2: Filtraggio Basato su Timestamp

Elabora solo dati dopo un tempo specifico

```ts
import { from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface LogEntry {
  timestamp: Date;
  message: string;
}

const startTime = new Date('2025-01-01T12:00:00');

const logs$ = from([
  { timestamp: new Date('2025-01-01T10:00:00'), message: 'Log 1' },
  { timestamp: new Date('2025-01-01T11:00:00'), message: 'Log 2' },
  { timestamp: new Date('2025-01-01T12:00:00'), message: 'Log 3' },
  { timestamp: new Date('2025-01-01T13:00:00'), message: 'Log 4' }
] as LogEntry[]);

logs$.pipe(
  skipWhile(log => log.timestamp < startTime)
).subscribe(log => {
  console.log(`[${log.timestamp.toISOString()}] ${log.message}`);
});
// Output:
// [2025-01-01T12:00:00.000Z] Log 3
// [2025-01-01T13:00:00.000Z] Log 4
```

### Pattern 3: Skip Basato su Stato

Salta fino a quando il sistema è pronto

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

interface SystemState {
  tick: number;
  isReady: boolean;
  data: number;
}

// Simulazione stato sistema
interval(500).pipe(
  take(10),
  map(i => ({
    tick: i,
    isReady: i >= 3, // Pronto dopo 3 secondi
    data: Math.floor(Math.random() * 100)
  } as SystemState)),
  skipWhile(state => !state.isReady)
).subscribe(state => {
  console.log(`Tick ${state.tick}: data=${state.data}`);
});
// Output: Solo dati da Tick 3 in poi
```

## 📚 Operatori Correlati

- **[takeWhile](/it/guide/operators/filtering/takeWhile)** - Prendi valori solo mentre la condizione è soddisfatta
- **[skip](/it/guide/operators/filtering/skip)** - Salta primi N valori
- **[skipLast](/it/guide/operators/filtering/skipLast)** - Salta ultimi N valori
- **[skipUntil](/it/guide/operators/filtering/skipUntil)** - Salta fino a quando un altro Observable si attiva
- **[filter](/it/guide/operators/filtering/filter)** - Passa solo valori che corrispondono alla condizione

## Riepilogo

L'operatore `skipWhile` salta i valori mentre una condizione è soddisfatta ed emette tutti i valori successivi dal punto in cui la condizione diventa falsa.

- ✅ Ideale per saltare dati iniziali non necessari
- ✅ La condizione non viene rivalutata una volta che diventa falsa
- ✅ Utile per saltare periodi di riscaldamento o preparazione
- ✅ Può essere usato per saltare righe header
- ⚠️ A differenza di `filter`, la condizione viene valutata solo una volta
- ⚠️ Se tutti i valori soddisfano la condizione, non viene emesso nulla
- ⚠️ Continua fino al completamento dello stream sorgente
