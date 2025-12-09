---
description: "defer verzögert Observable-Erstellung bis zur Subscription. Bei jeder Subscription neue Evaluierung. Ideal für Zufallswerte, aktuelle Zeit, API-Aufrufe."
---

# defer - Observable-Erstellung durch verzögerte Evaluierung

Der `defer`-Operator führt eine Observable-Factory-Funktion **zum Zeitpunkt der Subscription** aus und gibt das resultierende Observable zurück. Dadurch kann die Erstellung des Observables verzögert werden, bis tatsächlich subscribed wird.

## Grundlegende Syntax und Funktionsweise

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// Ausgabe:
// 0.8727962287400634
// 0.8499299688934545
```

In diesem Beispiel wird `Math.random()` bei jeder Subscription evaluiert, daher wird jedes Mal ein unterschiedlicher Wert emittiert.

[🌐 RxJS Offizielle Dokumentation - defer](https://rxjs.dev/api/index/function/defer)

## Typische Anwendungsbeispiele

Effektiv für APIs, externe Ressourcen, aktuelle Zeit oder Zufallszahlen - **Verarbeitung, deren Ergebnis sich je nach Ausführungszeitpunkt ändert**.

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// Ausgabe:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## Praktisches Codebeispiel (mit UI)

`defer` ist besonders nützlich für Verarbeitung mit Seiteneffekten oder Verarbeitung, die bei jedem Aufruf unterschiedliche Ergebnisse generiert.

Im folgenden Code können Sie durch Verwendung von `defer` die Bedeutung von "bei jeder Subscription ein unterschiedliches Observable generieren" erleben.
Besonders nützlich in **Fällen, in denen Sie jedes Mal einen Abrufvorgang durchführen möchten, anstatt einen Cache zu verwenden**.

### ✅ 1. Generierung einer zufälligen Zahl bei jedem Aufruf
```ts
import { defer, of } from 'rxjs';

// Observable, das eine zufällige Zahl generiert
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// UI-Elemente erstellen
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>Zufallswertgenerierung durch defer:</h3>';
document.body.appendChild(randomContainer);

// Generierungsbutton
const generateButton = document.createElement('button');
generateButton.textContent = 'Zufallswert generieren';
randomContainer.appendChild(generateButton);

// Verlaufsanzeigebereich
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// Button-Event
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `Generierter Wert: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// Erklärungstext
const randomExplanation = document.createElement('p');
randomExplanation.textContent = 'Jedes Mal, wenn Sie auf die Schaltfläche "Zufallswert generieren" klicken, wird ein neuer Zufallswert generiert. Bei Verwendung von normalem "of" würde der Wert nur einmal beim ersten Mal generiert, aber durch Verwendung von "defer" kann bei jeder Subscription ein neuer Wert generiert werden.';
randomContainer.appendChild(randomExplanation);
```

### ✅ 2. Ausführung von API-Anfragen bei jedem Aufruf

`defer` generiert bei jeder Subscription ein neues Observable, daher ist es **besonders effektiv in Situationen, in denen Sie unterschiedliche API-Anfragen je nach Benutzereingabe ausführen möchten**.
Zum Beispiel in folgenden Szenarien:

- ✅ Abruf mit unterschiedlichen URLs je nach dynamischen Queries oder Parametern
- ✅ Möchten Sie **jedes Mal die neuesten Daten abrufen** ohne Cache zu verwenden
- ✅ Möchten Sie Verarbeitung bei Ereignisauftreten verzögert evaluieren

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>API-Anfrage durch defer:</h3>';
document.body.appendChild(container);

// Eingabefeld
const input = document.createElement('input');
input.placeholder = 'Benutzer-ID eingeben';
container.appendChild(input);

// Ausführungsbutton
const button = document.createElement('button');
button.textContent = 'Benutzerinformationen abrufen';
container.appendChild(button);

// Ergebnisanzeige
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// Button-Event
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'Bitte geben Sie eine Benutzer-ID ein';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = 'Laden...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `Fehler: ${err.message}`),
  });
});
```

In diesem Beispiel wird durch `defer` `ajax.getJSON()` zu dem Zeitpunkt aufgerufen, zu dem der Benutzer den Button drückt,
**im Gegensatz zu Fällen wie `of(ajax.getJSON(...))`, wo von Anfang an evaluiert wird, können Sie den Ausführungszeitpunkt vollständig kontrollieren**.
