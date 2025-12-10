---
description: toArray è un operatore utility RxJS che combina tutti i valori emessi fino al completamento dell'Observable in un singolo array. È ideale per situazioni dove vuoi trattare l'intero stream come un array, come elaborazione batch, visualizzazione UI dopo acquisizione batch e elaborazione aggregata. Poiché accumula valori fino al completamento, non può essere usato con stream infiniti.
---

# toArray - Converti Valori in Array

L'operatore `toArray` è un operatore che **combina tutti i valori emessi dall'Observable fino al completamento in un singolo array**.
È utile per elaborazione batch, visualizzazione UI dopo recupero batch, aggregazione, ecc.


## 🔰 Sintassi e Operazione Base

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// Output:
// [1, 2, 3]
```

Tutti i valori vengono combinati in un singolo array, che viene emesso al completamento dell'Observable.

[🌐 Documentazione Ufficiale RxJS - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 Esempio di Utilizzo Tipico

Può essere usato in situazioni dove vuoi elaborare più risultati asincroni in una volta o emettere in batch alla UI.

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('Ricevi tutto al completamento:', result);
  });

// Output:
// Ricevi tutto al completamento: [0, 1, 2, 3, 4]
```


## 🧪 Esempio di Codice Pratico (con UI)

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// Area di visualizzazione output
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>Esempio toArray:</h3>';
document.body.appendChild(toArrayOutput);

// Area visualizzazione valori individuali
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>Valori Individuali:</h4>';
toArrayOutput.appendChild(individualValues);

// Area visualizzazione risultato array
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>Risultato Array:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// Sottoscrivi a valori individuali
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valore: ${val}`;
    individualValues.appendChild(valueItem);
  });

// Sottoscrivi allo stesso stream come array
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `Array risultato: [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // Visualizza elementi array individualmente
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ Riepilogo

- `toArray` **emette un array di tutti i valori al completamento**
- Ideale per situazioni dove vuoi gestire l'intero stream in aggregato
- Combinato con `concatMap`, `delay`, ecc., può essere usato per **elaborazione batch sequenziale asincrona**
