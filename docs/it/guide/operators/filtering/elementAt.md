---
description: "L'operatore elementAt è un operatore di filtraggio di RxJS che recupera solo i valori in una determinata posizione dell'indice. Funziona in modo simile all'accesso agli indici degli array."
---

# elementAt - Recuperato in base alla specifica dell'indice

L'operatore `elementAt' recupera **solo il valore nella posizione di indice specificata** dall'Observable e completa immediatamente il flusso. Funziona in modo simile a `array[index]` di un array.

## 🔰 Sintassi e utilizzo di base

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Uscita.: 30(Indice2Valore)
```

**Flusso delle operazioni**:.
1. 10 (indice 0) → saltare
2. 20 (indice 1) → salta
3. 30 (indice 2) → uscita e completamento
4. 40, 50 non valutati

[🌐 Documentazione ufficiale RxJS - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Tipico modello di utilizzo.

- **Paginazione**: ottenere il primo elemento di una pagina specifica.
- Ottenere dati garantiti dall'ordine**: ottenere l'ennesimo evento o messaggio.
- **Test e debug**: convalidare il valore di una posizione specifica.
- **Accesso simile a un array**: trattare Observable come un array.

## 🧠 Esempio pratico di codice 1: Conto alla rovescia degli eventi

Questo è un esempio di esecuzione di un'azione all'ennesimo clic.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICreare
const output = document.createElement('div');
output.innerHTML = '<h3>5Fare clic una volta per visualizzare il messaggio</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Fare clic su';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'di più5Fare clic una volta';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Evento click
const clicks$ = fromEvent(button, 'click');

// Per visualizzare il conteggio
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `di più${remaining}Fare clic una volta`;
  } else {
    counter.textContent = '';
  }
});

// 5Seconda volta (indice)4Click rilevati di
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Raggiunto！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Il quinto clic (indice 4) completa l'azione.
- Inizia da 0, proprio come l'indice dell'array.

## 🎯 Esempio pratico di codice 2: Ottenere il numero N dal flusso di dati.

Questo è un esempio di recupero di un ordine specifico di valori da dati pubblicati a intervalli regolari.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICreare
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Dal flusso di datiNOttenere il secondo';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Inserire l'indice (0〜dal flusso di dati (9)';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = 'Recuperare';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Flusso di dati (0.5I valori vengono emessi ogni secondo,10fino a 1)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0〜dal flusso di dati (9Inserire un intervallo di';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Indice ${index} Il valore viene recuperato...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ Recupero riuscito</strong><br>
        Indice: ${data.index}<br>
        Valore: ${data.value}<br>
        Orario: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ Errore: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- Recupera i valori a un indice specificato da un flusso pubblicato ogni 0,5 secondi.
- Viene generato un errore se l'indice è fuori dall'intervallo.

## 🆚 Confronto con operatori simili

### elementAt vs take vs first

##CODE_4___

{\AN8}```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Uscita.: 30(Indice2Valore)
---
description: elementAtオペレーターは、指定されたインデックス位置の値のみを取得するRxJSフィルタリングオペレーターです。配列のインデックスアクセスに似た動作をします。
---


## ⚠️ Note.

### 1. se l'indice è fuori dall'intervallo

Se l'indice specificato non viene raggiunto prima del completamento dello stream, viene generato un errore.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3Solo uno

numbers$.pipe(
  elementAt(5) // Indice5Richiesta
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Uscita.: Errore: no elements in sequence
```

### 2. Specificare i valori predefiniti.

Per evitare errori, è possibile specificare dei valori predefiniti.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Specificare un valore predefinito
numbers$.pipe(
  elementAt(5, 999) // Indice5Se non presente, restituisce999Restituisce un
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Uscita.: 999
```

### 3. Utilizzo con flussi asincroni

Nei flussi asincroni, attendere che venga raggiunta la posizione dell'indice.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1Emette un valore ogni secondo
interval(1000).pipe(
  elementAt(3) // Indice3(4(secondo valore)
).subscribe(console.log);
// 3Uscita dopo pochi secondi: 3
```

### 4. Gli indici negativi non sono ammessi

Non è possibile specificare indici negativi.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Gli indici negativi sono errori
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Errore: ArgumentOutOfRangeError: index out of range
```

Usare takeLast o last per arrivare alla fine dell'array.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Ottenere l'ultimo valore
numbers$.pipe(
  last()
).subscribe(console.log);
// Uscita.: 50

// ✅ Ottieni l'ultimo valoreNOttieni l'ultimo valore
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Uscita.: 40, 50
```

## 📚 Operatori correlati.

- **[take](. /take)** - N preso dall'inizio.
- **[first](. /first)** - ottiene il primo valore.
- **[last](. /last)** - ottiene l'ultimo valore.
- **[skip](. /skip)** - salta i primi N valori.
- **[takeLast](. /takeLast)** - ottiene gli ultimi N valori

## Riepilogo.

L'operatore elementAt recupera solo il valore nella posizione di indice specificata.

- Stesso comportamento dell'accesso all'indice dell'array.
- Ideale per recuperare l'ennesimo valore.
- ✅ È possibile specificare valori predefiniti per evitare errori.
- ⚠️ Errore se l'indice è fuori dall'intervallo (nessun valore predefinito)
- ⚠️ Non sono ammessi indici negativi
- ⚠️ I flussi asincroni attendono finché non vengono raggiunti
