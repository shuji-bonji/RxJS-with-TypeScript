---
description: "L'opérateur defer permet de retarder la fonction d'usine de l'Observable jusqu'au moment de l'abonnement. Ceci est utile lorsque vous souhaitez évaluer une valeur ou un processus différent à chaque abonnement, comme l'heure actuelle, des valeurs aléatoires, des requêtes API dynamiques."
---

# defer - Création d'un Observable avec évaluation différée

L'opérateur `defer` exécute la fonction de fabrique d'Observable au **moment de l'abonnement** et renvoie l'Observable résultant. Cela vous permet de retarder la création d'un Observable jusqu'à ce qu'il soit effectivement souscrit.

## Syntaxe et fonctionnement de base

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// Sortie:
// 0.8727962287400634
// 0.8499299688934545
```

Dans cet exemple, `Math.random()` est évalué pour chaque abonnement, donc une valeur différente est émise à chaque fois.

[🌐 Documentation officielle RxJS - defer](https://rxjs.dev/api/index/function/defer)

## Exemples d'applications typiques

Ceci est utile lorsque vous souhaitez exécuter des **processus** tels que des APIs, des ressources externes, l'heure actuelle, des nombres aléatoires, etc., dont les résultats varient en fonction du moment de l'exécution.

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// Sortie:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## Exemples de code pratique (avec interface utilisateur)

`defer` est particulièrement utile pour les processus qui ont des effets de bord ou qui produisent des résultats différents à chaque fois.

Dans le code ci-dessous, vous pouvez expérimenter ce que signifie utiliser `defer` pour "générer un Observable différent à chaque abonnement".
C'est particulièrement utile dans les cas où vous voulez faire le processus de récupération **à chaque fois** au lieu de le mettre en cache.

### 1. Générer un nombre aléatoire à chaque fois

```ts
import { defer, of } from 'rxjs';

// Observable qui génère des nombres aléatoires
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// Créer les éléments d'interface utilisateur
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>Génération de valeur aléatoire avec defer:</h3>';
document.body.appendChild(randomContainer);

// Bouton de génération
const generateButton = document.createElement('button');
generateButton.textContent = 'Générer une valeur aléatoire';
randomContainer.appendChild(generateButton);

// Zone d'affichage de l'historique
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// Événement du bouton
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `Valeur générée: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// Texte d'explication
const randomExplanation = document.createElement('p');
randomExplanation.textContent = 'Chaque fois que vous cliquez sur le bouton "Générer une valeur aléatoire", une nouvelle valeur aléatoire sera générée. Si vous utilisez of normal, la valeur sera générée une seule fois au début, mais en utilisant defer, vous pouvez générer une nouvelle valeur à chaque fois.';
randomContainer.appendChild(randomExplanation);
```

### 2. Exécuter chaque requête API

Parce que `defer` crée un nouvel Observable à chaque abonnement, il est particulièrement utile dans les situations où vous voulez exécuter différentes requêtes API basées sur **l'entrée de l'utilisateur, etc.**.
Par exemple, utilisez les scénarios suivants.

- ✅ Récupération à différentes URLs selon les requêtes dynamiques ou les paramètres
- ✅ Récupération des données les plus récentes à chaque fois sans utiliser le cache
- ✅ Évaluer paresseusement le traitement lorsqu'un événement se produit

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>Requête API avec defer:</h3>';
document.body.appendChild(container);

// Champ de saisie
const input = document.createElement('input');
input.placeholder = 'Entrez l\'ID utilisateur';
container.appendChild(input);

// Bouton d'exécution
const button = document.createElement('button');
button.textContent = 'Obtenir les informations utilisateur';
container.appendChild(button);

// Affichage du résultat
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// Événement du bouton
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'Veuillez entrer l\'ID utilisateur';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = 'Chargement...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `Erreur: ${err.message}`),
  });
});
```

Dans cet exemple, le `defer` fait en sorte que `ajax.getJSON()` soit appelé lorsque l'utilisateur appuie sur le bouton.
Contrairement à `of(ajax.getJSON(...))` qui évalue depuis le début, vous avez un contrôle total sur le timing de l'exécution.
