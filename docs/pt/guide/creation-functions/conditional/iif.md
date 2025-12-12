---
description: O operador iif é um operador de ramificação condicional do RxJS que seleciona um de dois Observables dependendo de uma expressão condicional, e pode ser usado como um operador ternário.
titleTemplate: ':title | RxJS'
---

# iif - Seleção de Observable baseada em condição

O operador `iif` seleciona um de dois Observables com base no resultado da avaliação de uma expressão condicional.
O operador ternário do JavaScript (`condition ? trueValue : falseValue`).


## Sintaxe básica e operação

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('SIM'), of('NÃO'));
}

getData(true).subscribe(console.log);

// Saída:
// SIM
```

Retorna `'SIM'` se a condição for `true`, `'NÃO'` se a condição for `false`.

[🌐 Documentação Oficial RxJS - iif](https://rxjs.dev/api/index/function/iif)

## Exemplos de Aplicação Típicos

`iif` é frequentemente usado em combinação com `EMPTY` para retornar um "stream sem emissão" se a condição não for atendida.

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`Valor positivo: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// Saída:
// Valor positivo: 1
```


## Exemplos de código prático (com UI)

O seguinte exemplo de código com UI usa `iif` para alternar dinamicamente o que publicar e se deve ou não publicar um Observable em resposta a ações do usuário e entrada numérica.
O seguinte exemplo de código com UI usa `iif` para alternar dinamicamente o que é emitido ou não emitido pelo Observable de acordo com a operação do usuário ou entrada numérica.

Esse padrão é adequado para os seguintes casos de uso práticos.

- ✅ Suprimir solicitações de API com base em valores de entrada (por exemplo, não enviar se o número for menor que 0)
- ✅ Alternar exibição de tela e modo de processamento de acordo com sinalizadores de configuração
- ✅ Reconhecimento e controle modal com base em condições

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`Valor positivo: ${value}`), EMPTY);
}

// Retornar diferentes Observables com base em condições
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('A condição é verdadeira'), of('A condição é falsa'));
}

// Criar elementos de UI
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>Exemplo do operador iif:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'Executar com condição Verdadeira';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'Executar com condição Falsa';
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

// Exemplo combinando com EMPTY (ramificação condicional por número)
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>Combinação de iif e EMPTY:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = 'Digite um número';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = 'Executar';
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
          'Um valor de 0 ou menos foi inserido, então nada foi emitido';
        emptyResult.style.color = 'gray';
      }
    },
  });
});
```
