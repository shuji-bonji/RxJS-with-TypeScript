---
description: "iif wählt eines von zwei Observables basierend auf Bedingung. Ähnlich dem ternären Operator, evaluiert bei Subscription. TypeScript-Beispiele mit EMPTY."
---

# iif - Auswahl von Observables basierend auf Bedingung

Der `iif`-Operator wählt eines von zwei Observables basierend auf dem Evaluierungsergebnis eines Bedingungsausdrucks.
Er hat eine ähnliche Funktionalität wie der ternäre Operator in JavaScript (`condition ? trueValue : falseValue`).


## Grundlegende Syntax und Funktionsweise

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('YES'), of('NO'));
}

getData(true).subscribe(console.log);

// Ausgabe:
// YES
```

Wenn die Bedingung `true` ist, wird `'YES'` zurückgegeben, wenn `false`, wird `'NO'` zurückgegeben.

[🌐 RxJS Offizielle Dokumentation - iif](https://rxjs.dev/api/index/function/iif)

## Typische Anwendungsbeispiele

`iif` wird oft in Kombination mit `EMPTY` verwendet, um einen "Stream, der nichts emittiert" zurückzugeben, wenn die Bedingung nicht erfüllt ist.

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`Positive Zahl: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// Ausgabe:
// Positive Zahl: 1
```


## Praktisches Codebeispiel (mit UI)

Im folgenden Codebeispiel mit UI werden der emittierte Inhalt des Observables und ob es emittiert wird dynamisch durch `iif` basierend auf Benutzerinteraktionen und numerischen Eingaben umgeschaltet.

Solche Muster eignen sich für folgende praktische Anwendungsfälle:

- ✅ API-Anfragen basierend auf Eingabewerten unterdrücken (z.B. nicht senden, wenn die Zahl <= 0)
- ✅ Bildschirmanzeige oder Verarbeitungsmodus basierend auf Konfigurationsflags umschalten
- ✅ Bestätigungsanzeige oder modale Steuerung basierend auf Bedingungen

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`Positive Zahl: ${value}`), EMPTY);
}

// Gibt unterschiedliche Observables basierend auf Bedingung zurück
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('Bedingung ist true'), of('Bedingung ist false'));
}

// UI-Elemente erstellen
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>Beispiel für iif-Operator:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'Mit True-Bedingung ausführen';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'Mit False-Bedingung ausführen';
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

// Kombinationsbeispiel mit EMPTY (bedingte Verzweigung nach Zahl)
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>Kombination von iif und EMPTY:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = 'Zahl eingeben';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = 'Ausführen';
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
          'Da ein Wert <= 0 eingegeben wurde, wurde nichts emittiert';
        emptyResult.style.color = 'gray';
      }
    },
  });
});

```
