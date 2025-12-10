---
description: L'operatore timestamp aggiunge un timestamp a ogni valore e registra l'ora in cui il valore è stato emesso, che può essere usato per misurazione performance e debugging.
---

# timestamp - Aggiungi Timestamp

L'operatore `timestamp` **aggiunge un timestamp** a ogni valore nello stream. Può essere usato per misurazione performance, debugging e analisi di serie temporali degli eventi registrando l'ora esatta in cui il valore è stato emesso.

## 🔰 Sintassi e Operazione Base

Converte ogni valore in un oggetto con timestamp.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// Output:
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

L'oggetto restituito ha la seguente struttura:
- `value`: Il valore originale
- `timestamp`: Timestamp (tempo Unix in millisecondi)

[🌐 Documentazione Ufficiale RxJS - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 Esempi di Utilizzo Tipici

- **Misurazione performance**: Misura tempo di elaborazione
- **Analisi timing eventi**: Misura intervalli tra azioni utente
- **Debugging e logging**: Registrazione del timing di emissione valori
- **Registrazione dati di serie temporali**: Storage con timestamp di dati sensore, ecc.

## 🧪 Esempio di Codice Pratico 1: Misurazione Intervalli Click

Questo è un esempio di misurazione dell'intervallo di click utente.

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// Creazione UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - Misurazione intervallo click';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Per favore clicca';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

let clickCount = 0;

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.insertBefore(logItem, output.firstChild);  // Visualizza il più recente in alto
}

fromEvent(button, 'click')
  .pipe(
    timestamp(),
    pairwise(),
    map(([prev, curr]) => {
      const interval = curr.timestamp - prev.timestamp;
      return {
        clickNumber: clickCount + 1,
        interval: interval,
        timestamp: new Date(curr.timestamp).toLocaleTimeString('it-IT')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? 'Click veloce!' :
                  data.interval < 1000 ? 'Normale' : 'Lento';

    addLog(
      `${data.clickNumber}° click: ${data.interval}ms intervallo [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('Per favore clicca il bottone (intervallo misurato dal 2° click)', '#e3f2fd');
```

- Misura accurata dell'intervallo click
- Visualizzazione codificata per colore secondo la velocità
- Registra l'ora di occorrenza con timestamp

## 🧪 Esempio di Codice Pratico 2: Misurazione Tempo di Elaborazione

Questo è un esempio di misurazione del tempo impiegato per ogni processo.

```ts
import { interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// Creazione UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - Misurazione tempo elaborazione';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.fontSize = '12px';
  logItem.style.fontFamily = 'monospace';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

addLog2('Elaborazione avviata...');

interval(500)
  .pipe(
    take(5),
    timestamp(),  // Timestamp prima dell'elaborazione
    map(data => {
      const start = data.timestamp;

      // Simula elaborazione pesante (tempo di elaborazione casuale)
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('it-IT', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `Valore${result.value}: inizio=${result.startTime}, tempo elaborazione=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- Tutta l\'elaborazione completata ---');
    }
  });
```

- Registra l'ora di inizio di ogni valore
- Misura il tempo impiegato per l'elaborazione
- Usa per analisi performance

## Utilizzo dei Timestamp

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // Elaborazione usando timestamp
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// Output:
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ Note Importanti

### 1. Precisione del Timestamp

Poiché viene usato `Date.now()` di JavaScript, la precisione è in millisecondi.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// Eventi ad alta frequenza (intervallo 1ms)
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`Valore: ${data.value}, Timestamp: ${data.timestamp}`);
  });
// Potrebbe avere lo stesso timestamp
```

Se serve maggiore precisione, considera l'utilizzo di `performance.now()`.

### 2. Il Timestamp è al Momento dell'Emissione

Il timestamp è l'ora in cui il valore è stato emesso, non quando è stato generato.

```ts
import { of, delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // Ritardo di 1 secondo
    timestamp()       // Timestamp dopo il ritardo
  )
  .subscribe(console.log);
```

### 3. Cambiamento Struttura Oggetto

L'utilizzo di `timestamp` wrappa il valore in un oggetto.

```ts
import { of, timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // Accedi al valore originale con .value
  )
  .subscribe(console.log);
// Output: 2, 4, 6
```

## 📚 Operatori Correlati

- **[tap](./tap)** - Esegui effetti collaterali (per debugging)
- **[delay](./delay)** - Ritardo di tempo fisso
- **[timeout](./timeout)** - Controllo timeout

## ✅ Riepilogo

L'operatore `timestamp` dà un timestamp per ogni valore.

- ✅ Registra accuratamente l'ora in cui ogni valore viene emesso
- ✅ Utile per misurazione performance
- ✅ Permette analisi degli intervalli tra eventi
- ✅ Utile per debugging e logging
- ⚠️ Precisione in millisecondi
- ⚠️ I valori vengono wrappati in oggetti
- ⚠️ I timestamp sono al momento dell'emissione
