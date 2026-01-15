---
description: Der debounceTime-Operator gibt nach kontinuierlichen Ereignisausgaben nur dann den letzten Wert aus, wenn für die angegebene Zeit kein neuer Wert kam. Ideal zur Optimierung häufiger Eingaben wie Suchfelder oder Fenstergrößenänderungen.
---

# debounceTime - Letzter Wert nach Stille

Der `debounceTime`-Operator gibt den letzten Wert aus, wenn nach der Ausgabe eines Werts im Stream für die angegebene Zeit kein neuer Wert ausgegeben wurde.
Wird sehr häufig in Szenarien verwendet, in denen häufige Ereignisse wie Suchfelder unterdrückt werden sollen.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- Wenn nach einem Eingabeereignis innerhalb von 300ms keine weitere Eingabe erfolgt, wird dieser Wert ausgegeben.
- Hat den Effekt, dass kurz aufeinanderfolgende Ereignisse zusammengefasst werden.

[🌐 RxJS Offizielle Dokumentation - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)

## 💡 Typische Anwendungsmuster

- Anfrage senden, nachdem Benutzer Tippen im Suchfeld beendet hat
- Endgültige Größe bei Fenstergrößenänderungsereignissen abrufen
- Endposition bei Scroll-Ereignissen abrufen

## 🧠 Praktisches Codebeispiel (mit UI)

Wenn Zeichen in ein Suchfeld eingegeben werden, wird nach 300ms Eingabestopp eine Suchstartmeldung angezeigt.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

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
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `Suche nach „${value}" wurde gestartet`;
});
```

- Reagiert nicht sofort während der Eingabe,
- Startet die Suche 300ms nach Eingabestopp mit dem neuesten Eingabewert.
