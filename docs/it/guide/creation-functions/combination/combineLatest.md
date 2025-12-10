---
description: "combineLatest combina gli ultimi valori di più Observable: Essenziale per validazione form in tempo reale, sincronizzazione stato e dati dipendenti"
---

# combineLatest - combina gli ultimi valori

`combineLatest` è una Funzione di Creazione che **combina tutti gli ultimi valori da più Observable**.
Ogni volta che un nuovo valore viene emesso da uno qualsiasi degli Observable sorgente, il risultato di tutti gli ultimi valori viene combinato.

## Sintassi base e utilizzo

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// Output:
// C 1
// C 2
// C 3
```

- Dopo che ogni Observable ha emesso **almeno un valore**, viene emesso il valore combinato.
- Ogni volta che arriva un nuovo valore per uno dei due, la coppia più recente viene riemessa.

[🌐 Documentazione Ufficiale RxJS - `combineLatest`](https://rxjs.dev/api/index/function/combineLatest)


## Pattern di utilizzo tipici

- **Validazione in tempo reale di input form** (es., monitoraggio simultaneo di nome e indirizzo email)
- **Sincronizzazione stato di stream multipli** (es., integrazione di valori sensori e stato dispositivo)
- **Recupero dati con dipendenze** (es., combinazione di ID utente e ID configurazione)

## Esempi di codice pratici (con UI)

Combina e visualizza sempre l'ultimo stato dei due campi di input di un form.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico combineLatest:</h3>';
document.body.appendChild(output);

// Crea campi form
const nameInput = document.createElement('input');
nameInput.placeholder = 'Inserisci nome';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'Inserisci email';
document.body.appendChild(emailInput);

// Observable di ogni input
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// Combina gli ultimi valori di input
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>Nome:</strong> ${name}</div>
    <div><strong>Email:</strong> ${email}</div>
  `;
});
```

- Quando digiti in uno dei campi, gli **ultimi due stati di input** vengono visualizzati immediatamente.
- Il `startWith('')` viene usato per ottenere il risultato combinato dall'inizio.


## Operatori Correlati

- **[combineLatestWith](/it/guide/operators/combination/combineLatestWith)** - Versione Pipeable Operator (usata in pipeline)
- **[withLatestFrom](/it/guide/operators/combination/withLatestFrom)** - attiva solo lo stream principale
- **[zip](/it/guide/creation-functions/combination/zip)** - Funzione di Creazione che accoppia valori corrispondenti
