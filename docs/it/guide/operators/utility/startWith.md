---
description: L'operatore startWith inserisce il valore iniziale specificato prima che l'Observable emetta il valore, ed è adatto per l'inizializzazione dello stato e la visualizzazione iniziale della UI.
---

# startWith - Fornire Valore Iniziale

L'operatore `startWith` è un operatore per **emettere il valore iniziale specificato prima che l'Observable sorgente emetta il valore**.
È utilizzato per gestione stato, visualizzazione iniziale, valori placeholder, ecc.


## 🔰 Sintassi e Operazione Base

```ts
import { of } from 'rxjs';
import { startWith } from 'rxjs';

of('B', 'C').pipe(
  startWith('A')
).subscribe(console.log);
// Output:
// A
// B
// C
```

Così, `startWith` aggiunge `'A'` prima, seguito dai valori dell'Observable sorgente.

[🌐 Documentazione Ufficiale RxJS - startWith](https://rxjs.dev/api/index/function/startWith)

## 💡 Esempio di Utilizzo Tipico

È utile quando vuoi impostare valori iniziali per stati o contatori. Ecco un esempio di un contatore che inizia con un valore iniziale di `100`.

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

interval(1000)
  .pipe(
    startWith(-1), // Inserisci -1 prima
    scan((acc, curr) => acc + 1, 100), // Incrementa dal valore iniziale 100
    take(10) // Output 10 volte in totale
  )
  .subscribe(console.log);
// Output:
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
```


## 🧪 Esempio di Codice Pratico (con UI)

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

// Area di visualizzazione output
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>Esempio startWith:</h3>';
document.body.appendChild(startWithOutput);

// Area visualizzazione contatore
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// Area visualizzazione lista valori
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// Stream contatore (ogni 1 secondo)
interval(1000)
  .pipe(
    // Inizia con 100 prima
    startWith(-1),
    // Aggiungi 1 a ogni valore al valore precedente
    scan((acc, curr) => acc + 1, 100),
    // Termina dopo 10 volte
    take(10)
  )
  .subscribe((count) => {
    // Aggiorna visualizzazione contatore
    counterDisplay.textContent = count.toString();

    // Aggiungi valore alla lista
    const valueItem = document.createElement('div');

    if (count === 100) {
      valueItem.textContent = `Valore iniziale: ${count} (aggiunto con startWith)`;
      valueItem.style.color = 'blue';
    } else {
      valueItem.textContent = `Prossimo valore: ${count}`;
    }

    valuesList.appendChild(valueItem);
  });
```


## ✅ Riepilogo

- `startWith` è utile per situazioni dove vuoi **inserire un valore fisso prima**
- Comunemente usato per inizializzazione stato, placeholder UI, visualizzazione iniziale form, ecc.
- Usato in combinazione con `scan`, `combineLatest`, ecc. per **costruire le basi per la gestione stato**
