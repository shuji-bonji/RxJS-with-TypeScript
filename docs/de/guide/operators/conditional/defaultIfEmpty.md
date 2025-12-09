---
description: "Der defaultIfEmpty-Operator gibt einen Standardwert zurück, wenn das Observable keinen Wert emittiert hat. Erläutert praktische Anwendungsfälle wie Verarbeitung leerer API-Antworten, Ergänzung von Anfangswerten und Fallbacks bei fehlenden Suchergebnissen sowie typsichere Implementierung mit TypeScript."
---

# defaultIfEmpty - Standardwert bei leerem Stream

Der `defaultIfEmpty`-Operator ist ein Operator, der **einen festgelegten Standardwert emittiert, wenn ein Observable abgeschlossen wird, ohne einen Wert zu emittieren**.
Er wird verwendet, um leere Arrays oder leere API-Ergebnisse zu verarbeiten.

## 🔰 Grundlegende Syntax und Verhalten

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

from([]).pipe(
  defaultIfEmpty('Keine Werte vorhanden')
).subscribe(console.log);

// Ausgabe:
// Keine Werte vorhanden
```

In diesem Beispiel wird ein leeres Array mit `from` in ein Observable umgewandelt, und durch `defaultIfEmpty` wird `'Keine Werte vorhanden'` ausgegeben.

[🌐 RxJS Offizielle Dokumentation - defaultIfEmpty](https://rxjs.dev/api/index/function/defaultIfEmpty)

## 💡 Typische Anwendungsfälle

- Wenn der Benutzer nichts eingegeben hat
- Wenn die API ein leeres Ergebnis zurückgibt
- Wenn keine Werte eine Bedingung erfüllen

In solchen Fällen wird es verwendet, um **die Situation "nichts wurde zurückgegeben" zu ergänzen**.

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of(['A', 'B', 'C']).pipe(delay(500))
    : EMPTY.pipe(delay(500));
}

mockApiCall(false)
  .pipe(defaultIfEmpty('Keine Daten vorhanden'))
  .subscribe(console.log);

// Ausgabe:
// Keine Daten vorhanden
```

## 🧪 Praktische Codebeispiele (mit UI)

### ✅ 1. Verwendung zur Überprüfung leerer Arrays

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

// UI-Aufbau
const container = document.createElement('div');
container.innerHTML = '<h3>Beispiel für defaultIfEmpty-Operator:</h3>';
document.body.appendChild(container);

const emptyBtn = document.createElement('button');
emptyBtn.textContent = 'Leeres Array verarbeiten';
container.appendChild(emptyBtn);

const nonEmptyBtn = document.createElement('button');
nonEmptyBtn.textContent = 'Nicht-leeres Array verarbeiten';
container.appendChild(nonEmptyBtn);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

emptyBtn.addEventListener('click', () => {
  result.textContent = 'Verarbeitung läuft...';
  from([]).pipe(
    defaultIfEmpty('Keine Daten vorhanden')
  ).subscribe(value => {
    result.textContent = `Ergebnis: ${value}`;
  });
});

nonEmptyBtn.addEventListener('click', () => {
  result.textContent = 'Verarbeitung läuft...';
  from([1, 2, 3]).pipe(
    defaultIfEmpty('Keine Daten vorhanden')
  ).subscribe(value => {
    result.textContent = `Ergebnis: ${value}`;
  });
});
```

### ✅ 2. Ergänzung von Standardwerten bei leeren API-Ergebnissen

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of([
        { id: 1, name: 'Element 1' },
        { id: 2, name: 'Element 2' },
      ]).pipe(delay(1000))
    : EMPTY.pipe(delay(1000));
}

const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>API-Ergebnisverarbeitung mit defaultIfEmpty:</h3>';
document.body.appendChild(apiContainer);

const dataBtn = document.createElement('button');
dataBtn.textContent = 'Mit Daten';
dataBtn.style.marginRight = '10px';
apiContainer.appendChild(dataBtn);

const emptyBtn2 = document.createElement('button');
emptyBtn2.textContent = 'Ohne Daten';
apiContainer.appendChild(emptyBtn2);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
apiContainer.appendChild(output);

dataBtn.addEventListener('click', () => {
  output.textContent = 'Abrufen...';
  mockApiCall(true)
    .pipe(defaultIfEmpty('Keine Daten gefunden'))
    .subscribe({
      next: (val) => {
        if (Array.isArray(val)) {
          const ul = document.createElement('ul');
          val.forEach((item) => {
            const li = document.createElement('li');
            li.textContent = `${item.id}: ${item.name}`;
            ul.appendChild(li);
          });
          output.innerHTML = '<h4>Abrufergebnis:</h4>';
          output.appendChild(ul);
        } else {
          output.textContent = val;
        }
      },
    });
});

emptyBtn2.addEventListener('click', () => {
  output.textContent = 'Abrufen...';
  mockApiCall(false)
    .pipe(defaultIfEmpty('Keine Daten gefunden'))
    .subscribe({
      next: (val) => {
        output.textContent = val.toString();
      },
    });
});

```
