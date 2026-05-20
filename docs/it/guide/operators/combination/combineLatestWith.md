---
description: "combineLatestWith è un operatore di combinazione RxJS che combina l'Observable originale con i valori più recenti di altri Observable. È ideale per le situazioni in cui si desidera monitorare costantemente i valori più recenti di più flussi dipendenti, come la convalida in tempo reale degli input dei moduli, la sincronizzazione di più stati, l'aggiornamento in tempo reale dei risultati dei calcoli, ecc."
---

# combineLatestWith - combina i valori più recenti

L'operatore combineLatestWith combina tutti gli **ultimi valori** dell'Observable originale e di qualsiasi altro Observable specificato.
Ogni volta che viene emesso un nuovo valore da uno degli Observable, viene emesso un risultato che combina tutti gli ultimi valori.
Questa è la versione Pipeable Operator di Creation Function `combineLatest`.

## 🔰 Sintassi e uso di base

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Esempi di output:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Dopo che ogni Observable ha emesso **almeno un valore**, viene emesso il valore combinato.
- Ogni volta che arriva un nuovo valore da una parte o dall'altra, viene riemessa la coppia più recente.

[🌐 Documentazione ufficiale di RxJS - combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)

## 💡 Tipico modello di utilizzo.

- **Convalida in tempo reale degli input di un modulo**: monitoraggio costante dello stato più recente di più campi.
- **Sincronizzazione di più stati dipendenti**: combinazione di valori di configurazione e input dell'utente
- **Aggiornamento in tempo reale dei risultati dei calcoli**: calcolo immediato dei risultati da più valori di input

## 🧠 Esempi pratici di codice (con UI)

Esempio di calcolo dell'importo totale in tempo reale a partire dagli input di prezzo e quantità.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Creazione di aree di output
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith Esempi pratici di:</h3>';
document.body.appendChild(output);

// Creazione di campi di input
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = 'Prezzo unitario';
priceInput.value = '100';
document.body.appendChild(priceInput);

const quantityInput = document.createElement('input');
quantityInput.type = 'number';
quantityInput.placeholder = 'Quantità';
quantityInput.value = '1';
document.body.appendChild(quantityInput);

// Area di visualizzazione dei risultati
const result = document.createElement('div');
result.style.fontSize = '20px';
result.style.marginTop = '10px';
document.body.appendChild(result);

// di ogni inputObservable
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Calcolato combinando gli ultimi valori
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Importo totale: ¥${total.toLocaleString()}</strong>`;
  });
```

- Quando si inserisce uno dei due campi, il totale viene **immediatamente ricalcolato** dai due valori più recenti.
- Si usa `startWith()` per ottenere il risultato combinato dall'inizio.

## 🔄 Differenze con la Creation Function `combineLatest`.

### Differenze di base.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Esempi di output:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

### Esempi specifici di utilizzo

**Se si desiderano solo combinazioni semplici, la Creation Function è la soluzione migliore**.


```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Taro');
const lastName$ = of('Yamada');
const age$ = of(30);

// Semplice e di facile lettura
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first}3 (${age}Età)`);
});
// Uscita: Yamada Taro (30Età)
```

**Se si desidera aggiungere un processo di conversione al flusso principale, si consiglia di utilizzare Pipeable Operator**.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Ricerca per...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Tutti</option><option>Libri</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Mainstream: Parole chiave di ricerca
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // Dopo l'input300msAttendere
  startWith('')
);

// Sottocorrente: Selezione della categoria
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Tutti')
);

// ✅ Pipeable OperatorEdizione - Completa in una pipeline
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // Convertito in minuscolo
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Ricerca per: "${result.term}" Categoria: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation FunctionEdizione - Diventa ridondante
import { combineLatest } from 'rxjs';
combineLatest([
  searchTerm$.pipe(map(term => term.toLowerCase())),
  category$
]).pipe(
  map(([term, category]) => ({
    term,
    category,
    timestamp: new Date().toLocaleTimeString()
  }))
).subscribe(result => {
  output.textContent = `Ricerca per: "${result.term}" Categoria: ${result.category} [${result.timestamp}]`;
});
```

**Quando si combinano più valori di configurazione**.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Creazione di cursori
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Red: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Green: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blue: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// Mainstream: Red
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable OperatorEdizione - RedCombinare altri colori come principali
red$
  .pipe(
    combineLatestWith(
      fromEvent(greenSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      ),
      fromEvent(blueSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      )
    ),
    map(([r, g, b]) => `rgb(${r}, ${g}, ${b})`)
  )
  .subscribe(color => {
    colorBox.style.backgroundColor = color;
    colorBox.textContent = color;
    colorBox.style.display = 'flex';
    colorBox.style.alignItems = 'center';
    colorBox.style.justifyContent = 'center';
    colorBox.style.color = '#fff';
    colorBox.style.textShadow = '1px 1px 2px #000';
  });
```

### Riepilogo.

- **`combineLatest`**: ideale se si desidera semplicemente combinare più flussi.
- **`combineLatestWith`**: ideale se si vogliono combinare i valori più recenti di altri flussi mentre si trasforma o si elabora il flusso principale

## ⚠️ Note.

### Non viene emesso finché non sono disponibili i valori iniziali.

Non vengono emessi risultati finché tutti gli Observable non hanno emesso almeno un valore.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // Nessun valore emessoObservable
).subscribe(console.log);
// Nessuna uscita (perchéNEVERNessun output (perché)
```

Questo problema può essere risolto fornendo un valore iniziale con `startWith()`.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Uscita: [0, null] → [1, null] → [2, null]
```

### Attenzione alle frequenti riedizioni.

Se un flusso emette valori frequentemente, anche il risultato sarà riemesso frequentemente.

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// 100msFlusso emesso ogni
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// slow$ogni volta che viene emesso un valore,fast$viene emesso, viene combinato con l'ultimo valore di
// → Le prestazioni richiedono attenzione
```

### Gestione degli errori.

Se si verifica un errore in un Observable, l'intero processo termina con un errore.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Si verificano errori')).pipe(
      catchError((err: unknown) => of('Recupero'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// Uscita: [0, 'Recupero'] → [1, 'Recupero']
```

## 📚 Operatori correlati.

- **[combineLatest](/it/guide/creation-functions/combinazione/combineLatest)** - versione della Creation Function.
- **[withLatestFrom](/it/guide/operators/combinazione/withLatestFrom)** - attivato solo da mainstream.
- **[zipWith](/it/guide/operators/combinazione/zipWith)** - Coppia di valori corrispondenti
