---
description: "withLatestFrom est un opérateur qui, chaque fois que l'Observable principal émet une valeur, combine celle-ci avec la dernière valeur d'un autre flux. Peut être utilisé pour obtenir le dernier état lors de la soumission d'un formulaire, référencer les valeurs d'entrée lors d'un clic de bouton, etc."
---

# withLatestFrom - Dernière valeur combinée

L'opérateur `withLatestFrom` **émet chaque fois que le flux principal émet une valeur**,
en la combinant avec la **dernière valeur** d'un autre flux.


## Syntaxe de base et utilisation

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

clicks$
  .pipe(
    withLatestFrom(timer$),
    map(([click, timerValue]) => `Compteur au moment du clic: ${timerValue}`)
  )
  .subscribe(console.log);

// Sortie:
// Compteur au moment du clic: 1
// Compteur au moment du clic: 2
// Compteur au moment du clic: 2
// Compteur au moment du clic: 5

```

- L'Observable principal (ici les clics) déclenche l'émission,
- La **dernière valeur** de l'Observable secondaire (ici le compteur) est combinée à chaque fois.

[Documentation officielle RxJS - `withLatestFrom`](https://rxjs.dev/api/index/function/withLatestFrom)


## Patterns d'utilisation typiques

- **Obtenir le dernier état lors d'une action utilisateur**
- **Référencer les données en cache lors d'une requête**
- **Combinaison de données déclenchée par événement**


## Exemple de code pratique (avec UI)

Un exemple qui récupère la dernière valeur d'un champ de saisie toutes les 2 secondes.

```ts
import { fromEvent, interval } from 'rxjs';
import { map, startWith, withLatestFrom } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'withLatestFrom - Récupération de la dernière saisie toutes les 2 secondes:';
document.body.appendChild(title);

// Création du champ de saisie
const nameInput = document.createElement('input');
nameInput.placeholder = 'Entrez un nom';
document.body.appendChild(nameInput);

// Création de la zone de sortie
const output = document.createElement('div');
document.body.appendChild(output);

// Observable de saisie
const name$ = fromEvent(nameInput, 'input').pipe(
  map((e) => (e.target as HTMLInputElement).value),
  startWith('') // Émet une chaîne vide au départ
);

// Timer (déclenche toutes les 2 secondes)
const timer$ = interval(2000);

// Récupère la dernière valeur de saisie à chaque déclenchement du timer
timer$.pipe(withLatestFrom(name$)).subscribe(([_, name]) => {
  const item = document.createElement('div');
  item.textContent = `Récupération toutes les 2 sec: Nom: ${name}`;
  output.prepend(item);
});

```

- Pendant que l'utilisateur continue à saisir,
- **La dernière saisie est récupérée et affichée toutes les 2 secondes**.

