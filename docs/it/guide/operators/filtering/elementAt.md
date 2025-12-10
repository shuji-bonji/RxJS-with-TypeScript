---
description: L'operatore elementAt è un operatore di filtraggio RxJS che recupera solo il valore alla posizione di indice specificata. Si comporta in modo simile all'accesso tramite indice degli array.
---

# elementAt - Ottieni Valore all'Indice Specificato

L'operatore `elementAt` recupera **solo il valore alla posizione di indice specificata** da un Observable e completa immediatamente lo stream. Si comporta in modo simile a `array[index]`.

## 🔰 Sintassi e Utilizzo Base

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30 (valore all'indice 2)
```

**Flusso di operazione**:
1. 10 (indice 0) → Salta
2. 20 (indice 1) → Salta
3. 30 (indice 2) → Emetti e completa
4. 40, 50 non vengono valutati

[🌐 Documentazione Ufficiale RxJS - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Pattern di Utilizzo Tipici

- **Paginazione**: Ottieni primo elemento di una pagina specifica
- **Recupero dati ordinati**: Ottieni N-esimo evento o messaggio
- **Test e debug**: Verifica valore a posizione specifica
- **Accesso simile agli array**: Tratta Observable come un array

## 🧠 Esempio di Codice Pratico: Conto alla Rovescia Eventi

Esempio di esecuzione di un'azione all'N-esimo click.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// Crea UI
const output = document.createElement('div');
output.innerHTML = '<h3>Mostra messaggio al 5° click</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Click';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Clicca ancora 5 volte';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Evento click
const clicks$ = fromEvent(button, 'click');

// Visualizzazione contatore
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `Altri ${remaining} click`;
  } else {
    counter.textContent = '';
  }
});

// Rileva 5° click (indice 4)
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Raggiunto!';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Completa al 5° click (indice 4).
- Inizia da 0, come l'indice degli array.

## 🆚 Confronto con Operatori Simili

### elementAt vs take vs first

| Operatore | Valore Recuperato | Conteggio Output | Caso d'Uso |
|:---|:---|:---|:---|
| `elementAt(n)` | Solo valore all'indice n | 1 | Ottieni N-esimo valore |
| `take(n)` | Primi n valori | n | Ottieni primi N valori |
| `first()` | Primo valore | 1 | Ottieni primo |
| `skip(n) + first()` | Primo dopo aver saltato n | 1 | Stesso di elementAt (non raccomandato) |

## ⚠️ Note

### 1. Quando l'Indice è Fuori Range

Se l'indice specificato non viene raggiunto prima che lo stream completi, si verifica un errore.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // Solo 3 elementi

numbers$.pipe(
  elementAt(5) // Richiedi indice 5
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Output: Errore: no elements in sequence
```

### 2. Specificare Valore Predefinito

Puoi specificare un valore predefinito per prevenire errori.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Specifica valore predefinito
numbers$.pipe(
  elementAt(5, 999) // Restituisci 999 se l'indice 5 non esiste
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Output: 999
```

### 3. Uso con Stream Asincroni

Per stream asincroni, attende fino al raggiungimento della posizione dell'indice.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// Emetti valore ogni secondo
interval(1000).pipe(
  elementAt(3) // Indice 3 (4° valore)
).subscribe(console.log);
// Output dopo 3 secondi: 3
```

### 4. Indice Negativo Non Disponibile

Gli indici negativi non possono essere specificati.

Per ottenere dalla fine dell'array, usa `takeLast` o `last`.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Ottieni ultimo valore
numbers$.pipe(
  last()
).subscribe(console.log);
// Output: 50

// ✅ Ottieni ultimi N valori
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Output: 40, 50
```

## 📚 Operatori Correlati

- **[take](/it/guide/operators/filtering/take)** - Ottieni primi N valori
- **[first](/it/guide/operators/filtering/first)** - Ottieni primo valore
- **[last](/it/guide/operators/filtering/last)** - Ottieni ultimo valore
- **[skip](/it/guide/operators/filtering/skip)** - Salta primi N valori
- **[takeLast](/it/guide/operators/filtering/takeLast)** - Ottieni ultimi N valori

## Riepilogo

L'operatore `elementAt` recupera solo il valore alla posizione di indice specificata.

- ✅ Stesso comportamento dell'accesso tramite indice array
- ✅ Ideale per ottenere l'N-esimo valore
- ✅ Può evitare errori specificando valore predefinito
- ⚠️ Errore se l'indice è fuori range (senza valore predefinito)
- ⚠️ Indice negativo non disponibile
- ⚠️ Attende fino al raggiungimento della posizione per stream asincroni
