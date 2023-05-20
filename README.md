# Projeto-SCARA

Projeto final da disciplina de Robótica 2022.2 ministrada pelo professor Ícaro Araújo.


## Objetivo do projeto

O projeto proposto deve simular um robô SCARA capaz de realizar uma atividade do tipo *pick and place*, com uma cena disponível no simulador *Webots* (disponível na imagem abaixo). O robô SCARA deve pegar o objeto (pato de borracha) e colocá-lo em cima da caixa de papelão. Para isso, será necessário utilizar a API do simulador em *python* para implementar as funções que executem a atividade proposta.

![Cena do robo SCARA](scara\worlds\.world.jpg)

## Rodando o projeto

Para conseguir rodar a simulação do projeto, siga as seguintes etapas:

1. Baixar o simulador *Webots* compatível com o seu dispositivo, disponível em [Cyberbotics: Robotics simulation with Webots](https://cyberbotics.com/#download)
2. Clonar este repositório na sua máquina
3. Abrir o arquivo scara\worlds\world.wbt no simulador
4. Iniciar simulação da cena no botão `play`

As funções responsáveis pelo funcionamento do SCARA estão disponíveis no arquivo 'scara\controllers\scara_controller\scara_controller.py' .
