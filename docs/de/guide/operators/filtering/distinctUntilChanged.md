---
description: Der distinctUntilChanged-Operator überspringt aufeinanderfolgende gleiche Werte und gibt nur Werte aus, die sich geändert haben, was eine effiziente Datenverarbeitung ermöglicht.
---

# distinctUntilChanged - Duplikate ignorieren

Der `distinctUntilChanged`-Operator entfernt Duplikate, wenn derselbe Wert aufeinanderfolgend ausgegeben wird, und gibt nur dann einen neuen Wert aus, wenn er sich vom vorherigen Wert unterscheidet.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Ausgabe: 1, 2, 3, 1, 2, 3
```

- Wenn der Wert mit dem vorherigen übereinstimmt, wird er ignoriert.
- Keine Stapelverarbeitung wie `Array.prototype.filter`, sondern **sequenzielle Bewertung**.

[🌐 RxJS Offizielle Dokumentation - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)


## 💡 Typische Anwendungsmuster

- Bei Formulareingabe-Erkennung unnötige Anfragen vermeiden, wenn derselbe Eingabewert aufeinanderfolgt
- Änderungserkennung in Sensor- oder Ereignisstreams
- Vermeidung unnötiger UI-Neuzeichnungen in der Zustandsverwaltung


## 🧠 Praktisches Codebeispiel (mit UI)

Simulation, bei der eine API-Anfrage **nur gesendet wird, wenn der eingegebene Text vom vorherigen unterschiedlich ist**.

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Ausgabebereich erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Suchbegriff eingeben';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Eingabestream
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `Suchwert: ${keyword} wird ausgeführt`;
  });

```

- Wenn sich die Eingabezeichen nicht ändern, wird keine Anfrage gesendet.
- Kann für effiziente Suchverarbeitung oder API-Kommunikationsoptimierung genutzt werden.
