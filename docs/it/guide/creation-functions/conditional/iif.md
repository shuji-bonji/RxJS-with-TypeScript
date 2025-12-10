---
description: L'operatore iif è un operatore di ramificazione condizionale RxJS che seleziona uno di due Observable in base a un'espressione condizionale, e può essere usato come un operatore ternario.
titleTemplate: ':title | RxJS'
---

# iif - Selezione di Observable in base a condizione

L'operatore `iif` seleziona uno di due Observable in base al risultato della valutazione di un'espressione condizionale.
L'operatore ternario JavaScript (`condition ? trueValue : falseValue`).


## Sintassi base e funzionamento

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('SI'), of('NO'));
}

getData(true).subscribe(console.log);

// Output:
// SI
```

Restituisce `'SI'` se la condizione è `true`, `'NO'` se la condizione è `false`.

[🌐 Documentazione Ufficiale RxJS - iif](https://rxjs.dev/api/index/function/iif)

## Esempi di Applicazione Tipici

`iif` viene spesso usato in combinazione con `EMPTY` per restituire uno "stream senza emissioni" se la condizione non è soddisfatta.

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`Valore positivo: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// Output:
// Valore positivo: 1
```


## Esempi di codice pratici (con UI)

Il seguente esempio di codice con UI usa `iif` per cambiare dinamicamente cosa pubblicare e se pubblicare o meno un Observable in risposta ad azioni utente e input numerici.
Il seguente esempio di codice con UI usa `iif` per cambiare dinamicamente cosa viene emesso o non emesso dall'Observable in base a operazioni utente o input numerici.

Tale pattern è adatto ai seguenti casi d'uso pratici.

- ✅ Sopprimere richieste API in base ai valori di input (es., non inviare se il numero è minore di 0)
- ✅ Cambiare visualizzazione schermo e modalità elaborazione in base a flag di configurazione
- ✅ Controllo conferma e modal in base a condizioni

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`Valore positivo: ${value}`), EMPTY);
}

// Restituisce Observable diversi in base a condizioni
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('Condizione è vera'), of('Condizione è falsa'));
}

// Crea elementi UI
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>Esempio operatore iif:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'Esegui con condizione True';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'Esegui con condizione False';
iifContainer.appendChild(falseButton);

const iifResult = document.createElement('div');
iifResult.style.marginTop = '10px';
iifResult.style.padding = '10px';
iifResult.style.border = '1px solid #ddd';
iifContainer.appendChild(iifResult);

trueButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(true).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'green';
  });
});

falseButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(false).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'red';
  });
});

// Esempio combinato con EMPTY (ramificazione condizionale per numero)
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>Combinazione di iif e EMPTY:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = 'Inserisci un numero';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = 'Esegui';
emptyContainer.appendChild(checkButton);

const emptyResult = document.createElement('div');
emptyResult.style.marginTop = '10px';
emptyResult.style.padding = '10px';
emptyResult.style.border = '1px solid #ddd';
emptyContainer.appendChild(emptyResult);

checkButton.addEventListener('click', () => {
  const value = Number(valueInput.value);
  emptyResult.textContent = '';

  conditionalData(value).subscribe({
    next: (result) => {
      emptyResult.textContent = result;
      emptyResult.style.color = 'green';
    },
    complete: () => {
      if (!emptyResult.textContent) {
        emptyResult.textContent =
          'È stato inserito un valore di 0 o meno, quindi nulla è stato emesso';
        emptyResult.style.color = 'gray';
      }
    },
  });
});
```
