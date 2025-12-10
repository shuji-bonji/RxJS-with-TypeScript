---
description: L'operatore defer fa sì che la funzione factory dell'Observable venga ritardata fino al momento della subscription. Questo è utile quando vuoi valutare un valore o processo diverso ogni volta che ti sottoscrivi, come l'ora corrente, valori casuali, richieste API dinamiche, o altri processi i cui risultati cambiano al momento dell'esecuzione.
titleTemplate: ':title | RxJS'
---

# defer - Creazione Observable con valutazione ritardata

L'operatore `defer` esegue la funzione factory dell'Observable **al momento della subscription** e restituisce l'Observable risultante. Questo ti permette di ritardare la creazione di un Observable fino a quando non viene effettivamente sottoscritto.

## Sintassi base e funzionamento

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// Output:
// 0.8727962287400634
// 0.8499299688934545
```

In questo esempio, `Math.random()` viene valutato per ogni subscription, quindi un valore diverso viene emesso ogni volta.

[🌐 Documentazione Ufficiale RxJS - defer](https://rxjs.dev/api/index/function/defer)

## Esempi di Applicazione Tipici

Questo è utile quando vuoi eseguire **processi** come API, risorse esterne, ora corrente, numeri casuali, ecc., i cui risultati variano in base al timing di esecuzione.

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// Output:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## Esempi di codice pratici (con UI)

`defer` è particolarmente utile per processi che hanno effetti collaterali o producono risultati diversi ogni volta.

Nel codice sotto, puoi sperimentare cosa significa usare `defer` per "generare un Observable diverso ogni volta che viene sottoscritto".
Questo è particolarmente utile nei casi** in cui vuoi eseguire il processo di fetch **ogni volta** invece di usare la cache.

### ✅ 1. generare un numero casuale ogni volta
```ts
import { defer, of } from 'rxjs';

// Observable che genera numeri casuali
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// Crea elementi UI
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>Generazione valore casuale con defer:</h3>';
document.body.appendChild(randomContainer);

// Pulsante genera
const generateButton = document.createElement('button');
generateButton.textContent = 'Genera valore casuale';
randomContainer.appendChild(generateButton);

// Area visualizzazione cronologia
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// Evento pulsante
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `Valore generato: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// Testo esplicativo
const randomExplanation = document.createElement('p');
randomExplanation.textContent = 'Ogni volta che clicchi il pulsante "Genera valore casuale", verrà generato un nuovo valore casuale. Se usi of normale, il valore viene generato solo una volta all\'inizio, ma usando defer, puoi generare un nuovo valore ogni volta.';
randomContainer.appendChild(randomExplanation);
```

### ✅ 2. Esegui ogni richiesta API

Poiché `defer` crea un nuovo Observable ogni volta che viene sottoscritto, è particolarmente utile in situazioni in cui vuoi eseguire richieste API diverse in base a **input utente, ecc.**.
Ad esempio, usa il seguente scenario.

- ✅ Fetching a URL diversi in base a query o parametri dinamici
- ✅ Fetching degli ultimi dati ogni volta** senza usare la cache
- ✅ Vuoi valutare lazily l'elaborazione quando si verifica un evento

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>Richiesta API con defer:</h3>';
document.body.appendChild(container);

// Campo input
const input = document.createElement('input');
input.placeholder = 'Inserisci ID utente';
container.appendChild(input);

// Pulsante esegui
const button = document.createElement('button');
button.textContent = 'Ottieni informazioni utente';
container.appendChild(button);

// Visualizzazione risultato
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// Evento pulsante
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'Inserisci ID utente';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = 'Caricamento...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `Errore: ${err.message}`),
  });
});
```

In questo esempio, il `defer` fa sì che `ajax.getJSON()` venga chiamato quando l'utente preme il pulsante,
**`of(ajax.getJSON(...))` A differenza del `defer`, che valuta dall'inizio, hai controllo completo** sul timing dell'esecuzione.
