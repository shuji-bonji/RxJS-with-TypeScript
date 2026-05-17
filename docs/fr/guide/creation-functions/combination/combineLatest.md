---
description: "combineLatest combine les dernières valeurs de plusieurs Observables : Essentiel pour la validation de formulaire en temps réel, la synchronisation d'état et les données dépendantes"
---

# combineLatest - combiner les dernières valeurs

`combineLatest` est une fonction de création qui **combine toutes les dernières valeurs de plusieurs Observables**.
Chaque fois qu'une nouvelle valeur est émise par l'un des Observables sources, le résultat de toutes les dernières valeurs est combiné.

## Syntaxe de base et utilisation

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// Sortie:
// C 1
// C 2
// C 3
```

- Après que chaque Observable a émis **au moins une valeur**, la valeur combinée est produite.
- Chaque fois qu'une nouvelle valeur est émise par l'un ou l'autre des Observables, la paire la plus récente est ré-émise.

[🌐 Documentation officielle RxJS - `combineLatest`](https://rxjs.dev/api/index/function/combineLatest)


## Modèles d'utilisation typiques

- **Validation en temps réel des entrées de formulaire** (par exemple, surveillance simultanée du nom et de l'adresse e-mail)
- **Synchronisation de l'état de plusieurs flux** (par exemple, intégration des valeurs des capteurs et de l'état de l'appareil)
- **Récupération de données avec dépendances** (par exemple, combinaison de l'identifiant utilisateur et de l'identifiant de configuration)

## Exemples de code pratique (avec interface utilisateur)

Toujours combiner et afficher le dernier état des deux champs de saisie d'un formulaire.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs';

// Créer une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de combineLatest:</h3>';
document.body.appendChild(output);

// Créer les champs de formulaire
const nameInput = document.createElement('input');
nameInput.placeholder = 'Entrez le nom';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'Entrez l\'email';
document.body.appendChild(emailInput);

// Observable de chaque entrée
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// Combiner les dernières valeurs d'entrée
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>Nom:</strong> ${name}</div>
    <div><strong>Email:</strong> ${email}</div>
  `;
});
```

- Lorsque vous tapez dans l'un des deux champs, les **deux derniers états de saisie** sont immédiatement affichés.
- La fonction `startWith('')` est utilisée pour obtenir le résultat combiné depuis le début.

> [!WARNING] Attention en code de production
> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)


## Opérateurs associés

- **[combineLatestWith](/fr/guide/operators/combination/combineLatestWith)** - Version opérateur Pipeable (utilisée dans le pipeline)
- **[withLatestFrom](/fr/guide/operators/combination/withLatestFrom)** - ne déclenche que le flux principal
- **[zip](/fr/guide/creation-functions/combination/zip)** - fonction de création d'appariement de valeurs correspondantes
