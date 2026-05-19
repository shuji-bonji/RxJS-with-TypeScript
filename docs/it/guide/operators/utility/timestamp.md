---
description: "L'operatore timestamp aggiunge un timestamp a ogni valore, registrando l'ora in cui è stato emesso, che può essere utilizzata per la misurazione delle prestazioni e il debug."
---

# timestamp - timestamping

L'operatore `timestamp` **assegna un timestamp** a ogni valore nello stream. Registrando l'ora esatta in cui il valore è stato emesso, può essere utilizzato per la misurazione delle prestazioni, il debug e l'analisi delle serie temporali degli eventi.

## 🔰 Sintassi e funzionamento di base

Converte ogni valore in un oggetto con data e ora.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// Uscita:
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

L'oggetto restituito ha la seguente struttura.
- `valore`: il valore originale
- `timestamp`: timestamp (tempo Unix in millisecondi)

[🌐 Documentazione ufficiale RxJS - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 Caso d'uso tipico.

- **Misurazione delle prestazioni**: misurazione dei tempi di elaborazione
- Analisi dei tempi degli eventi**: misurazione degli intervalli tra le azioni dell'utente
- **Debug e logging**: registrazione dei valori emessi.
- **Registrazione di dati in serie temporali**: memorizzazione con data e ora di dati di sensori, ad esempio.

## 🧪 Esempio pratico di codice 1: misurazione degli intervalli di clic

Questo esempio mostra come misurare l'intervallo di clic dell'utente.

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// UICreare
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - Fare clic sulla misurazione dell'intervallo';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Fare clic su';
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
  output.insertBefore(logItem, output.firstChild);  // Mostra il più recente in alto
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
        timestamp: new Date(curr.timestamp).toLocaleTimeString('ja-JP')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? 'Clic più veloce!' :
                  data.interval < 1000 ? 'Normale' : 'Lentamente';

    addLog(
      `${data.clickNumber}2° clic: ${data.interval}msIntervallo [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('Fare clic sul pulsante (2Misura l'intervallo dalla seconda volta)', '#e3f2fd');
```

- Misurazione accurata dell'intervallo di clic
- Visualizzazione con codice colore in base alla velocità
- I timbri temporali registrano l'ora in cui si verifica il clic

## 🧪 Esempio pratico di codice 2: Misurazione dei tempi di elaborazione

Questo esempio mostra come misurare il tempo impiegato per ogni processo.

```ts
import { interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// UICreare
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - Misurazione del tempo di elaborazione';
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

addLog2('Inizio dell'elaborazione...');

interval(500)
  .pipe(
    take(5),
    timestamp(),  // Tempo prima dell'elaborazione
    map(data => {
      const start = data.timestamp;

      // Simulare un'elaborazione pesante (tempi di elaborazione casuali)
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('ja-JP', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `Valore${result.value}: Inizio=${result.startTime}, Tempo di elaborazione=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- Tutta l'elaborazione è stata completata ---');
    }
  });
```

- Registrare l'ora di inizio dell'elaborazione per ogni valore
- Misurazione del tempo impiegato per ogni processo
- Utilizzato per l'analisi delle prestazioni

## 🧪 Esempio pratico di codice 3: Registro eventi

Questo è un esempio di registrazione di tutti gli eventi con un timestamp.

```ts
import { merge, fromEvent, interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// UICreare
const container3 = document.createElement('div');
container3.style.marginTop = '20px';
document.body.appendChild(container3);

const title3 = document.createElement('h3');
title3.textContent = 'timestamp - Registro eventi';
container3.appendChild(title3);

const clickButton = document.createElement('button');
clickButton.textContent = 'Cliccare';
clickButton.style.marginRight = '10px';
container3.appendChild(clickButton);

const hoverDiv = document.createElement('div');
hoverDiv.textContent = 'Passare il mouse qui';
hoverDiv.style.display = 'inline-block';
hoverDiv.style.padding = '10px';
hoverDiv.style.border = '2px solid #4CAF50';
hoverDiv.style.cursor = 'pointer';
container3.appendChild(hoverDiv);

const log3 = document.createElement('div');
log3.style.marginTop = '10px';
log3.style.border = '1px solid #ccc';
log3.style.padding = '10px';
log3.style.maxHeight = '200px';
log3.style.overflow = 'auto';
log3.style.fontFamily = 'monospace';
log3.style.fontSize = '12px';
container3.appendChild(log3);

function addLog3(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.backgroundColor = color;
  logItem.style.padding = '2px';
  logItem.textContent = message;
  log3.insertBefore(logItem, log3.firstChild);
}

// Integrare più fonti di eventi
const events$ = merge(
  fromEvent(clickButton, 'click').pipe(map(() => 'CLICK')),
  fromEvent(hoverDiv, 'mouseenter').pipe(map(() => 'HOVER_IN')),
  fromEvent(hoverDiv, 'mouseleave').pipe(map(() => 'HOVER_OUT')),
  interval(3000).pipe(take(5), map(i => `TIMER_${i}`))
);

events$
  .pipe(
    timestamp()
  )
  .subscribe(data => {
    const time = new Date(data.timestamp).toLocaleTimeString('ja-JP', { hour12: false }) +
                 '.' + (data.timestamp % 1000).toString().padStart(3, '0');

    const colors: Record<string, string> = {
      'CLICK': '#c8e6c9',
      'HOVER_IN': '#fff9c4',
      'HOVER_OUT': '#ffccbc',
    };

    const color = data.value.startsWith('TIMER') ? '#e1bee7' :
                  (colors[data.value] || '#e3f2fd');

    addLog3(`[${time}] Evento: ${data.value}`, color);
  });

addLog3('Registro eventi in corso di registrazione...', '#e3f2fd');
```

- Integrare più fonti di eventi
- Timestamp di tutti gli eventi
- Traccia gli eventi in ordine cronologico

## Come possono essere utilizzati i timestamp

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // Elaborazione con timestamp
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// Uscita:
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ Note.

### 1. precisione del timestamp

Precisione al millisecondo dovuta all'uso di `Date.now()` di JavaScript.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// Eventi ad alta frequenza (1msintervallo)
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`Valore: ${data.value}, Timestamp: ${data.timestamp}`);
  });
// Può essere lo stesso timestamp
```

Considerare l'uso di `performance.now()` se è necessaria una maggiore precisione.

### 2. Il timestamp è al momento dell'emissione.

Il timestamp è il momento in cui il valore è stato emesso, non quando è stato generato.

```ts
import { of, delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // 1Ritardo di 1 secondo
    timestamp()       // Timestamp dopo il ritardo
  )
  .subscribe(console.log);
```

### 3. Modifiche alla struttura dell'oggetto

Utilizzando `timestamp`, i valori sono avvolti in oggetti.

```ts
import { of, timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // .valueAccesso al valore originale con
  )
  .subscribe(console.log);
// Uscita: 2, 4, 6
```

## 📚 Operatori correlati.

- **[tap](. /tap)** - Esegue effetti collaterali (per il debug).
- **[delay](. /delay)** - ritardo fisso.
- **[timeout](. /timeout)** - controllo del timeout.

## ✅ Sommario.

L'operatore `timestamp` assegna un timestamp a ogni valore.

- Registra con precisione l'ora di emissione di ogni valore.
- ✅ Utile per la misurazione delle prestazioni
- ✅ Consente l'analisi degli intervalli di eventi
- ✅ Utile per il debug e il logging
- ⚠️ Precisione al millisecondo
- ⚠️ I valori sono avvolti in oggetti
- ⚠️ I timestamp sono aggiornati al momento della pubblicazione
