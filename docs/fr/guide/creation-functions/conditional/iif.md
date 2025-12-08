---
description: "L'opérateur iif est un opérateur de branchement conditionnel RxJS qui sélectionne l'un des deux Observables en fonction d'une expression conditionnelle, et peut être utilisé comme un opérateur ternaire."
---

# iif - Sélection d'un Observable selon une condition

L'opérateur `iif` sélectionne l'un des deux Observables en fonction du résultat de l'évaluation d'une expression conditionnelle.
Similaire à l'opérateur ternaire JavaScript (`condition ? trueValue : falseValue`).


## Syntaxe et fonctionnement de base

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('OUI'), of('NON'));
}

getData(true).subscribe(console.log);

// Sortie:
// OUI
```

Retourne `'OUI'` si la condition est `true`, `'NON'` si la condition est `false`.

[🌐 Documentation officielle RxJS - iif](https://rxjs.dev/api/index/function/iif)

## Exemples d'applications typiques

`iif` est souvent utilisé en combinaison avec `EMPTY` pour retourner un "flux sans émission" si la condition n'est pas remplie.

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`Valeur positive: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// Sortie:
// Valeur positive: 1
```


## Exemples de code pratique (avec interface utilisateur)

L'exemple de code suivant avec interface utilisateur utilise `iif` pour basculer dynamiquement entre la publication et la non-publication d'un Observable en réponse aux actions de l'utilisateur et aux entrées numériques.

Un tel modèle est adapté aux cas d'utilisation pratiques suivants.

- ✅ Supprimer les requêtes API basées sur les valeurs d'entrée (par exemple, ne pas envoyer si le nombre est inférieur à 0)
- ✅ Changer l'affichage de l'écran et le mode de traitement selon les drapeaux de configuration
- ✅ Accusé de réception et contrôle modal selon les conditions

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`Valeur positive: ${value}`), EMPTY);
}

// Retourner différents Observables selon les conditions
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('La condition est vraie'), of('La condition est fausse'));
}

// Créer les éléments d'interface utilisateur
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>Exemple de l\'opérateur iif:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'Exécuter avec condition True';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'Exécuter avec condition False';
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

// Exemple de combinaison avec EMPTY (branchement conditionnel par nombre)
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>Combinaison de iif et EMPTY:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = 'Entrez un nombre';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = 'Exécuter';
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
          'Une valeur de 0 ou moins a été entrée, donc rien n\'a été émis';
        emptyResult.style.color = 'gray';
      }
    },
  });
});
```
