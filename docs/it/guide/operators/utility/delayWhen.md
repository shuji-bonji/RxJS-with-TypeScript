---
description: L'operatore delayWhen controlla dinamicamente il timing del ritardo di ogni valore con un Observable separato per ottenere un'elaborazione del ritardo flessibile secondo le condizioni.
---

# delayWhen - Controllo Ritardo Dinamico

L'operatore `delayWhen` determina dinamicamente il tempo di ritardo per ogni valore **con un Observable individuale**. Mentre l'operatore `delay` fornisce un ritardo di tempo fisso, `delayWhen` può applicare un ritardo diverso per ogni valore.

## 🔰 Sintassi e Operazione Base

Specifica una funzione che restituisce un Observable che determina il ritardo per ogni valore.

```ts
import { of, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    delayWhen(value => {
      const delayTime = value === 'B' ? 2000 : 1000;
      return timer(delayTime);
    })
  )
  .subscribe(console.log);
// Output:
// A (dopo 1 secondo)
// C (dopo 1 secondo)
// B (dopo 2 secondi)
```

In questo esempio, solo il valore `'B'` avrà un ritardo di 2 secondi applicato, gli altri avranno un ritardo di 1 secondo.

[🌐 Documentazione Ufficiale RxJS - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 Esempi di Utilizzo Tipici

- **Ritardo basato sul valore**: Cambia ritardo in base a priorità o tipo
- **Ritardo basato su eventi esterni**: Attendi interazione utente o completamento di altri stream
- **Ritardo condizionale**: Ritarda solo per un valore specifico
- **Controllo timing asincrono**: Attendi risposta API o prontezza dati

## 🧪 Esempio di Codice Pratico 1: Ritardo per Priorità

Questo è un esempio di controllo del timing di elaborazione secondo la priorità del task.

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// Creazione UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - Ritardo basato su priorità';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

interface Task {
  id: number;
  name: string;
  priority: 'high' | 'medium' | 'low';
}

const tasks: Task[] = [
  { id: 1, name: 'Task A', priority: 'low' },
  { id: 2, name: 'Task B', priority: 'high' },
  { id: 3, name: 'Task C', priority: 'medium' },
  { id: 4, name: 'Task D', priority: 'high' },
  { id: 5, name: 'Task E', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('it-IT', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('Elaborazione avviata', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // Imposta tempo di ritardo secondo la priorità
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // Alta priorità: 0.5 secondi
          break;
        case 'medium':
          delayTime = 1500; // Media priorità: 1.5 secondi
          break;
        case 'low':
          delayTime = 3000; // Bassa priorità: 3 secondi
          break;
      }
      return timer(delayTime);
    })
  )
  .subscribe({
    next: task => {
      const colors = {
        high: '#c8e6c9',
        medium: '#fff9c4',
        low: '#ffccbc'
      };
      addLog(
        `Elaborazione ${task.name} (priorità: ${task.priority})`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('Tutti i task completati', '#e3f2fd');
    }
  });
```

- I task ad alta priorità vengono elaborati dopo 0.5 secondi
- I task a media priorità vengono elaborati dopo 1.5 secondi, bassa priorità dopo 3 secondi
- Realizza ordine di elaborazione secondo l'importanza del task

## 🧪 Esempio di Codice Pratico 2: Ritardo per Eventi Esterni

Questo è un esempio di attesa di un click utente prima di emettere un valore.

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// Creazione UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - Attesa click';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'Clicca per visualizzare prossimo valore';
button.style.marginBottom = '10px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.minHeight = '100px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

let clickCount = 0;

of('Messaggio 1', 'Messaggio 2', 'Messaggio 3')
  .pipe(
    tap(msg => {
      addLog2(`In attesa: ${msg} (per favore clicca il bottone)`);
      button.textContent = `Clicca per visualizzare "${msg}"`;
    }),
    delayWhen(() => {
      // Ritarda fino a quando si verifica l'evento click
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ Visualizzato: ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = 'Completato';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- Tutti i messaggi visualizzati ---');
    }
  });
```

- Ogni valore viene emesso dopo aver atteso un click utente
- È possibile il controllo del ritardo innescato da eventi esterni
- Può essere applicato a elaborazione di sequenze interattive

## 🆚 Confronto con delay

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - ritardo di tempo fisso
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// Tutti i valori ritardati di 1 secondo

// delayWhen - ritardo diverso per valore
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1 dopo 1 secondo, 2 dopo 2 secondi, 3 dopo 3 secondi
```

| Operatore | Controllo Ritardo | Caso d'Uso |
|:---|:---|:---|
| `delay` | Tempo fisso | Semplice ritardo uniforme |
| `delayWhen` | Dinamico (per valore) | Ritardo condizionale, attesa eventi esterni |

## ⚠️ Note Importanti

### 1. L'Observable di Ritardo è Generato Nuovo Ogni Volta

```ts
// ❌ Esempio sbagliato: Riuso della stessa istanza Observable
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // Non funzionerà dalla 2a volta
).subscribe();

// ✅ Esempio corretto: Genera nuovo Observable ogni volta
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. Quando l'Observable di Ritardo Non Completa

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ Esempio sbagliato: Restituire NEVER ritarda per sempre
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // I valori non verranno emessi
  )
  .subscribe(console.log);
// Nessun output
```

L'Observable di ritardo deve sempre emettere un valore o completare.

### 3. Gestione Errori

Se si verifica un errore nell'Observable di ritardo, l'intero stream andrà in errore.

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('Errore ritardo'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Errore:', err.message)
  });
// Output: 1
// Errore: Errore ritardo
```

## 📚 Operatori Correlati

- **[delay](./delay)** - Ritardo di tempo fisso
- **[debounceTime](../filtering/debounceTime)** - Ritardo dopo che l'input si ferma
- **[throttleTime](../filtering/throttleTime)** - Passa valore ogni periodo fisso
- **[timeout](./timeout)** - Controllo timeout

## ✅ Riepilogo

L'operatore `delayWhen` controlla dinamicamente il timing del ritardo per ogni valore.

- ✅ Ritardi diversi possono essere applicati a ogni valore
- ✅ Controllo ritardo tramite eventi esterni e Observable
- ✅ Regola timing elaborazione in base a priorità e tipo
- ⚠️ L'Observable di ritardo deve essere generato nuovo ogni volta
- ⚠️ L'Observable di ritardo deve completare o emettere un valore
- ⚠️ Fai attenzione alla gestione errori
